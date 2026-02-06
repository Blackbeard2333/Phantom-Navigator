# Step 1: Camera-Lidar Calibration (see launch file)
# Step 2: 2D UAV Detection Using YOLO

## Purpose: Obtain an initial spatial cue for UAV location from image.
## Output: 2D bbox of detected drone

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import torch
import numpy as np
import cv2
import warnings
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesis, ObjectHypothesisWithPose, BoundingBox2D
from geometry_msgs.msg import Pose2D

# Suppress specific torch warning
warnings.filterwarnings("ignore", category=FutureWarning, message="`torch.cuda.amp.autocast.*")

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10)
        
        # The bounding box 
        self.bbox_pub = self.create_publisher(Detection2DArray, '/visual_detection/detections', 10)

        # The image with bounding box
        self.image_pub = self.create_publisher(Image, '/visual_detection/bbox_image', 10)

        # The projected points in lidar frame (for visualization purpose only)
        self.detection_pub = self.create_publisher(PointStamped, '/visual_detection/visual_detections_lidar_frame', 10)
        self.bridge = CvBridge()

        self.depth_image = None


        # Load YOLOv5 model (nano for speed)
        # self.model = torch.hub.load(
        #     '/home/cpsl/Documents/CPSL_UAV_Tracking/yolov5',
        #     'custom',
        #     path='yolov5n.pt',
        #     source='local'
        # )

        # Load YOLOv5 model (custom trained for drones)
        path_1 ='/home/cpsl/Documents/CPSL_UAV_Tracking/yolov5/runs/train/drone_yolov5n3/weights/best.pt'
        self.model = torch.hub.load(
            '/home/cpsl/Documents/CPSL_UAV_Tracking/yolov5',
            'custom',
            # path='runs/train/drone_yolov5n3/weights/best.pt',
            path = path_1,
            source='local'
        )
        self.model.conf = 0.45  # confidence threshold

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.target_frame = 'cpsl_uav_1/livox_frame'  # Lidar frame

        self.get_logger().info("YOLOv5 detector node initialized.")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f"Depth CV Bridge error: {e}")

    def image_callback(self, msg):
        if self.depth_image is None:
            return
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_image)
            detections = results.pandas().xyxy[0]

            for _, det in detections.iterrows():
                label = det['name']
                conf = det['confidence']
                x1, y1, x2, y2 = map(int, [det['xmin'], det['ymin'], det['xmax'], det['ymax']])

                # Draw bounding box
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(cv_image, f"{label} {conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

                # Estimate depth from center
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                if 0 <= cx < self.depth_image.shape[1] and 0 <= cy < self.depth_image.shape[0]:
                    depth_raw = self.depth_image[cy, cx]
                    # Handle 16UC1 and 32FC1 encodings
                    if self.depth_image.dtype == np.uint16:
                        z = depth_raw / 1000.0  # mm to meters
                    elif self.depth_image.dtype == np.float32:
                        z = float(depth_raw)
                    else:
                        self.get_logger().warn("Unsupported depth image encoding.")
                        continue

                    if 0.1 < z < 20.0:
                        fx = 909.1417
                        fy = 908.6625
                        cx_intr = 637.8266
                        cy_intr = 356.8405

                        x = (cx - cx_intr) * z / fx
                        y = (cy - cy_intr) * z / fy

                        point_camera = PointStamped()
                        point_camera.header = msg.header
                        point_camera.point.x = x
                        point_camera.point.y = y
                        point_camera.point.z = z

                        try:
                            point_lidar = self.tf_buffer.transform(
                                point_camera,
                                self.target_frame,
                                timeout=rclpy.duration.Duration(seconds=1.0))
                            self.get_logger().info(
                                f"3D detection in LiDAR frame: ({point_lidar.point.x:.2f}, {point_lidar.point.y:.2f}, {point_lidar.point.z:.2f})")
                            self.detection_pub.publish(point_lidar)
                        except Exception as e:
                            self.get_logger().warn(f"TF transform failed: {e}")

            # Convert OpenCV image back to ROS Image message
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            img_msg.header = msg.header
            self.image_pub.publish(img_msg)

            # Publish the bounding box as detection arrays
            # Create the Detection2DArray
            detection_array = Detection2DArray()
            detection_array.header = msg.header  # use camera image header (contains timestamp and frame_id)

            for _, det in detections.iterrows():
                label = det['name']
                conf = float(det['confidence'])
                x1, y1, x2, y2 = map(float, [det['xmin'], det['ymin'], det['xmax'], det['ymax']])
                cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                w, h = x2 - x1, y2 - y1


                detection = Detection2D()
                detection.header = msg.header  # each detection inherits header

                # Bounding box
                bbox = BoundingBox2D()
                bbox.center.position.x = float(cx)
                bbox.center.position.y = float(cy)
                bbox.size_x = float(w)
                bbox.size_y = float(h)
                detection.bbox = bbox


                # Hypothesis (confidence score, object ID)
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = "drone"  # string
                hypothesis.hypothesis.score = conf
                detection.results.append(hypothesis)

                # Append to array
                detection_array.detections.append(detection)

            # Publish
            self.bbox_pub.publish(detection_array)

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    rclpy.spin(node)
    rclpy.shutdown()



# class YOLODetector(Node):
#     def __init__(self):
#         super().__init__('yolo_detector')
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/camera/color/image_raw',
#             self.image_callback,
#             10)
#         self.depth_subscription = self.create_subscription(
#             Image,
#             '/camera/camera/depth/image_rect_raw',
#             self.depth_callback,
#             10)
#         self.image_pub = self.create_publisher(Image, '/visual_detection/bbox_image', 10)
#         self.detection_pub = self.create_publisher(PointStamped, '/visual_detection/visual_detections_lidar_frame', 10)
#         self.bridge = CvBridge()

#         self.depth_image = None






