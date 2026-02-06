import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np

class LiDARObjectDetector(Node):
    def __init__(self):
        super().__init__('lidar_object_detector')
        self.declare_parameter('use_temporal_smoothing', True)
        self.use_temporal_smoothing = self.get_parameter('use_temporal_smoothing').get_parameter_value().bool_value

        self.previous_centroids = {}  # For temporal smoothing

        self.subscription = self.create_subscription(
            PointCloud2,
            '/cpsl_uav_1/livox/lidar',
            self.listener_callback,
            10)
        self.marker_pub = self.create_publisher(MarkerArray, 'uav_tracking/objects_markers', 10)
        self.get_logger().info('LiDAR Object Detector Node has started.')

    def listener_callback(self, msg):
        points = np.array([
            [pt[0], pt[1], pt[2]]
            for pt in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ])

        if points.shape[0] == 0:
            self.get_logger().info("Empty point cloud.")
            return

        db = DBSCAN(eps=0.3, min_samples=30).fit(points)
        labels = db.labels_
        unique_labels = set(labels) - {-1}

        self.get_logger().info(f"Detected {len(unique_labels)} raw objects before filtering.")

        marker_array = MarkerArray()
        min_box_volume = 0.001  # 10cm x 10cm x 10cm

        for i, label in enumerate(unique_labels):
            cluster = points[labels == label]
            if len(cluster) < 30:
                continue

            min_pt = np.min(cluster, axis=0)
            max_pt = np.max(cluster, axis=0)
            scale = max_pt - min_pt

            volume = scale[0] * scale[1] * scale[2]
            if volume < min_box_volume:
                continue

            centroid = cluster.mean(axis=0)
            if self.use_temporal_smoothing:
                prev = self.previous_centroids.get(label, centroid)
                centroid = 0.8 * prev + 0.2 * centroid
                self.previous_centroids[label] = centroid

            marker = Marker()
            marker.header = msg.header
            marker.ns = "uav_objects"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float((min_pt[0] + max_pt[0]) / 2.0)
            marker.pose.position.y = float((min_pt[1] + max_pt[1]) / 2.0)
            marker.pose.position.z = float((min_pt[2] + max_pt[2]) / 2.0)
            marker.pose.orientation.w = 1.0
            marker.scale.x = float(scale[0])
            marker.scale.y = float(scale[1])
            marker.scale.z = float(scale[2])
            marker.color.r = 0.2
            marker.color.g = 0.8
            marker.color.b = 0.4
            marker.color.a = 0.7
            marker.lifetime.sec = 0

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = LiDARObjectDetector()
    rclpy.spin(node)
    rclpy.shutdown()
