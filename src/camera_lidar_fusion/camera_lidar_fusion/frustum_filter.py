# Step 3: 3D Frustum Filtering using Projected LiDAR Points

## Purpose: Reduce the full LiDAR cloud to a region of interest (ROI) likely containing the UAV.
## Output: Filtered point clouds

import rclpy
from rclpy.node import Node
import message_filters
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import tf2_ros
from scipy.spatial.transform import Rotation as R

class FrustumFilter(Node):
    def __init__(self):
        super().__init__('frustum_filter')
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.pc_sub = message_filters.Subscriber(self, PointCloud2, '/cpsl_uav_1/livox/lidar')
        self.info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/camera/color/camera_info')
        self.bbox_sub = message_filters.Subscriber(self, Detection2DArray, '/visual_detection/detections')

        ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.pc_sub, self.info_sub, self.bbox_sub], 10, 0.2
        )
        ts.registerCallback(self.callback)

        # Publisher for filtered point cloud
        self.filtered_pub = self.create_publisher(PointCloud2, '/camera_lidar_fusion/frustum_filtered', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Add robustness for a relaxed and most recent bbox
        # self.last_valid_bboxes = []
        self.margin = 10  # pixel margin to relax bbox
        self.min_depth = 0.1  # adjust if UAV might be farther

        self.get_logger().info("Frustum Filter node initialized.")

    def callback(self, image_msg, cloud_msg, cam_info_msg, detection_array_msg):
        try:
            # Get intrinsics
            fx, fy = cam_info_msg.k[0], cam_info_msg.k[4]
            cx, cy = cam_info_msg.k[2], cam_info_msg.k[5]

            # Transform LiDAR points into camera frame
            transform = self.tf_buffer.lookup_transform(
                cam_info_msg.header.frame_id,
                cloud_msg.header.frame_id,
                rclpy.time.Time()
            )
            T = np.eye(4)
            trans = transform.transform.translation
            rot = transform.transform.rotation
            T[:3, :3] = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()
            T[:3, 3] = [trans.x, trans.y, trans.z]

            lidar_points = np.array([
                [p[0], p[1], p[2], 1.0] for p in point_cloud2.read_points(cloud_msg, field_names=["x", "y", "z"], skip_nans=True)
            ])

            if len(lidar_points) == 0:
                return

            cam_points = (T @ lidar_points.T).T  # (N, 4)
            img_coords = np.zeros((cam_points.shape[0], 2), dtype=int)
            img_coords[:, 0] = (cam_points[:, 0] * fx / cam_points[:, 2] + cx).astype(int)
            img_coords[:, 1] = (cam_points[:, 1] * fy / cam_points[:, 2] + cy).astype(int)

            height, width = image_msg.height, image_msg.width
            
            # # Save valid bounding boxes or reuse last valid ones
            # if detection_array_msg.detections:
            #     self.last_valid_bboxes = detection_array_msg.detections
            # bboxes_to_use = self.last_valid_bboxes

            # Only proceed if detections are available
            if not detection_array_msg.detections:
                self.get_logger().warn("No detection bounding boxes in current frame. Skipping frustum filtering.")
                return
            bboxes_to_use = detection_array_msg.detections


            # Frustum filtering without robustness considering
            # filtered = []
            # for i, (u, v, z) in enumerate(zip(img_coords[:, 0], img_coords[:, 1], cam_points[:, 2])):
            #     if z <= 0.1 or not (0 <= u < width and 0 <= v < height):
            #         continue
            #     for det in bboxes_to_use:
            #         box = det.bbox
            #         x_min = int(box.center.position.x - box.size_x / 2)
            #         x_max = int(box.center.position.x + box.size_x / 2)
            #         y_min = int(box.center.position.y - box.size_y / 2)
            #         y_max = int(box.center.position.y + box.size_y / 2)

            #         if x_min <= u <= x_max and y_min <= v <= y_max:
            #             filtered.append(tuple(lidar_points[i][:3]))
            #             break

            # Frustum filtering
            filtered = []
            for i, (u, v, z) in enumerate(zip(img_coords[:, 0], img_coords[:, 1], cam_points[:, 2])):
                if z <= self.min_depth or not (0 <= u < width and 0 <= v < height):
                    continue
                for det in bboxes_to_use:
                    box = det.bbox
                    x_min = int(box.center.position.x - box.size_x / 2) - self.margin
                    x_max = int(box.center.position.x + box.size_x / 2) + self.margin
                    y_min = int(box.center.position.y - box.size_y / 2) - self.margin
                    y_max = int(box.center.position.y + box.size_y / 2) + self.margin

                    if x_min <= u <= x_max and y_min <= v <= y_max:
                        filtered.append(tuple(lidar_points[i][:3]))
                        break

            # Publish filtered cloud
            header = cloud_msg.header
            filtered_cloud = point_cloud2.create_cloud_xyz32(header, filtered)
            self.filtered_pub.publish(filtered_cloud)

            self.get_logger().info(f"Frustum filter kept {len(filtered)} points.")

        except Exception as e:
            self.get_logger().error(f"Frustum filter failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FrustumFilter()
    rclpy.spin(node)
    rclpy.shutdown()
