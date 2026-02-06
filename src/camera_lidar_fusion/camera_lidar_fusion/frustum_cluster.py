# Step 4: 3D Clustering inside the Frustum
# Step 5: Association

## Purpose: Segment out distinct 3D blobs from the filtered points and select the UAV.
## Output: The estimated UAV cluster

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection2DArray
import numpy as np
from scipy.spatial import cKDTree
from collections import deque
from sklearn.cluster import DBSCAN
import open3d as o3d
import time


class FrustumCluster(Node):
    def __init__(self):
        super().__init__('frustum_cluster')

        # Parameters
        self.declare_parameter('clustering_method', 'dbscan')
        self.declare_parameter('selection_method', 'combined')
        self.declare_parameter('cluster_tolerance', 0.2)
        self.declare_parameter('min_cluster_size', 20)
        self.declare_parameter('max_cluster_size', 1000)

        # Subscribers
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera_lidar_fusion/frustum_filtered',
            self.cloud_callback,
            10
        )
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/visual_detection/detections',
            self.detection_callback,
            10
        )

        # Publisher
        self.cluster_pub = self.create_publisher(
            PointCloud2,
            '/camera_lidar_fusion/uav_cluster',
            10
        )

        self.last_uav_centroid = None
        self.last_uav_cluster_pts = None
        self.latest_bbox_center = None  # (x, y)

        self.has_published_cluster = False

        self.get_logger().info("Frustum Cluster node initialized.")

    def detection_callback(self, msg):
        if msg.detections:
            # Use the first detected box center as projection target
            box = msg.detections[0].bbox
            self.latest_bbox_center = (
                box.center.position.x,
                box.center.position.y
            )

    def cloud_callback(self, msg):
        # Load params
        method = self.get_parameter('clustering_method').get_parameter_value().string_value
        selector = self.get_parameter('selection_method').get_parameter_value().string_value
        tolerance = self.get_parameter('cluster_tolerance').get_parameter_value().double_value
        min_cluster = self.get_parameter('min_cluster_size').get_parameter_value().integer_value
        max_cluster = self.get_parameter('max_cluster_size').get_parameter_value().integer_value

        points = np.array([
            [p[0], p[1], p[2]] for p in point_cloud2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        ])
        if len(points) == 0:
            self.get_logger().warn("Empty filtered cloud.")
            return
        
        if self.latest_bbox_center is None:
            self.get_logger().warn("Skipping clustering — no bounding box available.")
            return

        # Clustering
        if method == 'kdtree':
            cluster_indices = self.cluster_with_kdtree(points, tolerance, min_cluster, max_cluster)
        elif method == 'dbscan':
            cluster_indices = self.cluster_with_dbscan(points, tolerance, min_cluster, max_cluster)
        else:
            self.get_logger().error(f"Unknown clustering method: {method}")
            return

        if not cluster_indices:
            self.get_logger().warn("No valid clusters found.")
            return

        # Cluster selection heuristic
        selected = self.select_cluster(points, cluster_indices, selector)
        if selected is None:
            self.get_logger().warn("No cluster passed the selection criteria.")
            return

        clustered_points = points[selected]
        output_cloud = point_cloud2.create_cloud_xyz32(msg.header, clustered_points)
        self.cluster_pub.publish(output_cloud)

        # Update temporal memory
        self.last_uav_centroid = np.mean(clustered_points, axis=0)
        self.last_uav_cluster_pts = clustered_points

        self.get_logger().info(f"Published UAV cluster with {len(clustered_points)} points using '{selector}' selection.")

        # Naive approach for rate control
        # time.sleep(2.0)  # Sleep for 2 seconds to simulate 0.5 Hz rate


    def cluster_with_kdtree(self, points, tolerance, min_size, max_size):
        tree = cKDTree(points)
        visited = np.zeros(len(points), dtype=bool)
        clusters = []

        for i in range(len(points)):
            if visited[i]:
                continue

            cluster = []
            queue = deque([i])
            visited[i] = True

            while queue:
                idx = queue.popleft()
                cluster.append(idx)
                neighbors = tree.query_ball_point(points[idx], tolerance)
                for n in neighbors:
                    if not visited[n]:
                        visited[n] = True
                        queue.append(n)

            if min_size <= len(cluster) <= max_size:
                clusters.append(cluster)

        return clusters

    def cluster_with_dbscan(self, points, tolerance, min_size, max_size):
        db = DBSCAN(eps=tolerance, min_samples=min_size).fit(points)
        labels = db.labels_
        unique = set(labels) - {-1}
        return [np.where(labels == u)[0].tolist() for u in unique if min_size <= (labels == u).sum() <= max_size]

    # Cluster selection principles
    ## 1. Projection Alignment Heuristics (2D Consistency)
    ## 2. 3D Proximity to Prior UAV Location (Temporal Heuristic)
    ## 3. Size & Shape Priors (Static Appearance Heuristics)
    ## 4. Combined Scoring Function (Combining above)

    def select_cluster(self, points, cluster_indices, method):
        best_total_score = -np.inf
        best_cluster = None
        best_scores = {}
        cluster_scores = []

        # Constants
        target_size = 100
        target_volume = 0.056
        # expected_extent = np.array([0.4, 0.4, 0.35]) # Holybro X500
        expected_extent = np.array([0.3, 0.3, 0.35]) # Holybro X500
        # expected_extent = np.array([0.2, 0.2, 0.05]) # DJI Tello
        max_2d_distance = 200.0
        max_3d_distance = 5.0
        MIN_PRIOR_SHAPE_SCORE = 0.5
        MIN_PREV_MATCH_SCORE = 0.5

        for cluster in cluster_indices:
            cluster_pts = points[cluster]
            centroid = np.mean(cluster_pts, axis=0)
            size = len(cluster)
            extent = np.maximum(np.max(cluster_pts, axis=0) - np.min(cluster_pts, axis=0), 1e-6)
            volume = np.prod(extent)

            # --- Prior shape score ---
            shape_extent_diff = np.linalg.norm(extent - expected_extent)
            norm_extent_error = shape_extent_diff / np.linalg.norm(expected_extent)
            norm_size_error = abs(size - target_size) / target_size
            norm_volume_error = abs(volume - target_volume) / target_volume
            prior_shape_score = 1.0 - min((0.6 * norm_extent_error + 0.3 * norm_volume_error + 0.1 * norm_size_error), 1.0)

            # --- Match score components ---
            centroid_score = shape_diff_score = volume_change_score = icp_score = 0.0
            if self.last_uav_cluster_pts is not None:
                prev_pts = self.last_uav_cluster_pts
                prev_centroid = np.mean(prev_pts, axis=0)

                centroid_dist = np.linalg.norm(centroid - prev_centroid)
                centroid_score = 1.0 - min(centroid_dist / max_3d_distance, 1.0)

                curr_extent = np.max(cluster_pts, axis=0) - np.min(cluster_pts, axis=0)
                prev_extent = np.max(prev_pts, axis=0) - np.min(prev_pts, axis=0)
                shape_diff = np.linalg.norm(curr_extent - prev_extent)
                shape_diff_score = 1.0 - min(shape_diff / 0.2, 1.0)

                curr_volume = np.prod(curr_extent)
                prev_volume = np.prod(prev_extent)
                vol_diff = abs(curr_volume - prev_volume) / max(prev_volume, 1e-6)
                volume_change_score = 1.0 - min(vol_diff / 0.5, 1.0)

                try:
                    source = o3d.geometry.PointCloud()
                    target = o3d.geometry.PointCloud()
                    source.points = o3d.utility.Vector3dVector(cluster_pts)
                    target.points = o3d.utility.Vector3dVector(prev_pts)
                    result = o3d.pipelines.registration.registration_icp(
                        source, target, 0.3, np.eye(4),
                        o3d.pipelines.registration.TransformationEstimationPointToPoint()
                    )
                    icp_rmse = result.inlier_rmse
                    icp_score = 1.0 - min(icp_rmse / 0.2, 1.0)
                except Exception as e:
                    self.get_logger().warn(f"ICP failed: {e}")
                    icp_score = 0.0

            match_score = 0.25 * centroid_score + 0.25 * shape_diff_score + 0.25 * volume_change_score + 0.25 * icp_score
            

            # --- Total score ---
            if method == 'prior':
                total_score = prior_shape_score
            elif method == 'combined':
                total_score = 0.5 * match_score + 0.5 * prior_shape_score
            else:
                self.get_logger().warn(f"Unknown selection method: {method}")
                continue

            cluster_scores.append({
                'cluster_idx': cluster,
                'centroid': centroid.tolist(),
                'size': size,
                'volume': float(volume),
                'prior_shape_score': prior_shape_score,
                'match_score': match_score,
                'total_score': total_score,
                'centroid_score': centroid_score,
                'shape_diff_score': shape_diff_score,
                'volume_change_score': volume_change_score,
                'icp_score': icp_score
            })

            if total_score > best_total_score:
                best_total_score = total_score
                best_cluster = cluster
                best_scores = {
                    'prior_score': prior_shape_score,
                    'match_score': match_score,
                    'total_score': total_score
                }

        self.get_logger().info(f"Top cluster total score: {best_total_score:.3f} | Prior: {best_scores.get('prior_score', 0):.2f}, Match: {best_scores.get('match_score', 0):.2f}")

        # --- Best cluster verification ---
        if not hasattr(self, 'has_published_cluster') or not self.has_published_cluster:
            if best_scores['prior_score'] < MIN_PRIOR_SHAPE_SCORE:
                self.get_logger().warn(f"No previous cluster — rejected due to low prior shape score ({best_scores['prior_score']:.3f})")
                return None
        else:
            if best_scores['prior_score'] < MIN_PRIOR_SHAPE_SCORE:
                self.get_logger().warn(f"Rejected due to low prior shape score ({best_scores['prior_score']:.3f})")
                return None
            if best_scores['match_score'] < MIN_PREV_MATCH_SCORE:
                self.get_logger().warn(f"Rejected due to low match score ({best_scores['match_score']:.3f})")
                return None

        self.has_published_cluster = True
        self.get_logger().info("Best cluster has been published.")
        return best_cluster


    def projection_alignment(self, centroid):
        # Simple projection to 2D image (assuming camera looks along Z)
        fx, fy, cx, cy = 909.1417, 908.6625, 637.8266, 356.8405  # If unknown, estimate
        x, y, z = centroid
        u = fx * x / z + cx
        v = fy * y / z + cy
        bbox_x, bbox_y = self.latest_bbox_center
        dist_2d = np.hypot(u - bbox_x, v - bbox_y)
        return -dist_2d  # smaller distance is better

def main(args=None):
    rclpy.init(args=args)
    node = FrustumCluster()
    rclpy.spin(node)
    rclpy.shutdown()
