import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
import message_filters
import numpy as np
import cv2
import tf2_ros
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R
from sensor_msgs_py import point_cloud2

class FusionProjector(Node):
    def __init__(self):
        super().__init__('fusion_projector')
        self.bridge = CvBridge()

        # Subscribers
        image_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        cloud_sub = message_filters.Subscriber(self, PointCloud2, '/cpsl_uav_1/livox/lidar')
        caminfo_sub = message_filters.Subscriber(self, CameraInfo, '/camera/camera/color/camera_info')

        ts = message_filters.ApproximateTimeSynchronizer([image_sub, cloud_sub, caminfo_sub], 10, 0.1)
        ts.registerCallback(self.callback)

        # Publisher
        self.image_pub = self.create_publisher(Image, '/fusion_projection/projected_image', 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("Fusion projector node initialized.")

    def project_pointcloud_to_image(self, points, cam_info):
        fx = cam_info.k[0]
        fy = cam_info.k[4]
        cx = cam_info.k[2]
        cy = cam_info.k[5]

        img_points = []
        for pt in points:
            x, y, z = pt
            if z <= 0.1:
                continue
            u = int((x * fx) / z + cx)
            v = int((y * fy) / z + cy)
            img_points.append((u, v))
        return img_points

    def callback(self, image_msg, cloud_msg, cam_info_msg):
        try:
            points = np.array([
                [pt[0], pt[1], pt[2]]
                for pt in point_cloud2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
            ])

            # TF lookup
            # transform = self.tf_buffer.lookup_transform(
            #     cam_info_msg.header.frame_id,
            #     cloud_msg.header.frame_id,
            #     rclpy.time.Time()
            # )

            # transform = self.tf_buffer.lookup_transform(
            #     target_frame='camera_color_optical_frame',  # <-- we want points in this frame
            #     source_frame='cpsl_uav_1/livox_frame',      # <-- we have points in this frame
            #     time=rclpy.time.Time()
            # )

            # With this (allow timeout):
            transform = self.tf_buffer.lookup_transform(
                cam_info_msg.header.frame_id,
                cloud_msg.header.frame_id,
                cloud_msg.header.stamp,  # use the timestamp of the point cloud
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            trans = transform.transform.translation
            rot = transform.transform.rotation

            r = R.from_quat([rot.x, rot.y, rot.z, rot.w])
            rot_matrix = r.as_matrix()

            tf_mat = np.eye(4)
            tf_mat[:3, :3] = rot_matrix
            tf_mat[:3, 3] = [trans.x, trans.y, trans.z]

            points_hom = np.hstack((points, np.ones((points.shape[0], 1))))
            points_cam = (tf_mat @ points_hom.T).T[:, :3]

            # Project
            img_points = self.project_pointcloud_to_image(points_cam, cam_info_msg)
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

            for (u, v) in img_points:
                if 0 <= u < cv_image.shape[1] and 0 <= v < cv_image.shape[0]:
                    cv2.circle(cv_image, (u, v), 2, (0, 255, 0), -1)

            # Publish
            projected_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            projected_msg.header = image_msg.header
            self.image_pub.publish(projected_msg)

        except Exception as e:
            self.get_logger().error(f"Projection failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FusionProjector()
    rclpy.spin(node)
    rclpy.shutdown()
