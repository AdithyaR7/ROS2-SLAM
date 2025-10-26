#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Header
from vehicle_interfaces.msg import SemanticObject, DetectionArray
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from ultralytics import YOLO
import numpy as np
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
 
class DetectionNode(Node):
    def __init__(self):
        super().__init__("detection_node")
        
        self.rgb_video_sub_ = self.create_subscription(
            Image, "/camera/camera/image_raw", self.callback_image_raw, 10)
        self.depth_video_sub_ = self.create_subscription(
            Image, "/camera/camera/depth/image_raw", self.callback_depth_raw, 10)
        self.camera_info_sub_ = self.create_subscription(
            CameraInfo, "/camera/camera/camera_info", self.callback_camera_info, 10)
        
        self.image_annotated_pub_ = self.create_publisher(Image, "image_annotated", 10)
        self.detection_array_pub_ = self.create_publisher(DetectionArray, "detection_array", 10)
        self.marker_array_pub_ = self.create_publisher(MarkerArray, "detection_markers", 10)

        # Initialize params
        self.bridge = CvBridge()                  # bridge to convert ros image to cv2 image
        self.rgb_image_annotated = []             # initialize rgb image from Image msg
        self.rgb_image_raw_header = Header()      # initialize header from Image msg
        self.depth_image_raw = []                 # initialize depth image from Image msg
        self.detection_array = DetectionArray()   # initialize detection array

        # Camera intrinsics
        self.fx = None                            # focal length x
        self.fy = None                            # focal length y
        self.cx = None                            # principal point x
        self.cy = None                            # principal point y
        self.camera_frame = "camera_optical_link" # camera frame id

        # TF2 for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) 

        # YOLO - Object Detection. Should use GPY by default
        self.model = YOLO('yolov10s.pt')                      # Load model once 
        self.model.overrides['verbose'] = False               # Disable YOLO logging
        self.model(np.zeros((640, 480, 3), dtype=np.uint8))   # Warm-up inference
        device = next(self.model.model.parameters()).device
        self.get_logger().info(f"YOLO model loaded on device: {device}")

        self.get_logger().info("Object Detection Node has been started")
        
                
    def publish_image_annotated(self):
        # get the ros message from a cv2 image. Copy over header.
        image_annotated_msg = self.bridge.cv2_to_imgmsg(self.rgb_image_annotated, encoding='bgr8')
        image_annotated_msg.header = self.rgb_image_raw_header
        self.image_annotated_pub_.publish(image_annotated_msg)  # publish image

    def run_detection(self, image):
        results = self.model(image)
        return results

    def create_annotated_image(self, results):
        annotated_image = results[0].plot()           # fast drawing of bounding boxes
        return annotated_image

    def pixel_to_camera_frame(self, u, v, depth):
        """Convert pixel coordinates to 3D point in camera frame"""
        if self.fx is None or depth <= 0:
            return None

        X = (u - self.cx) * depth / self.fx
        Y = (v - self.cy) * depth / self.fy
        Z = depth

        return Point(x=float(X), y=float(Y), z=float(Z))

    def transform_to_map_frame(self, point_camera):
        """Transform point from camera frame to map frame"""
        if point_camera is None:
            self.get_logger().warn("point_camera is None", throttle_duration_sec=5.0)
            return Point(x=0.0, y=0.0, z=0.0)

        try:
            # Create stamped point in camera frame
            point_stamped = PointStamped()
            point_stamped.header.frame_id = self.camera_frame
            point_stamped.header.stamp = self.rgb_image_raw_header.stamp
            point_stamped.point = point_camera

            # Transform to map frame (use latest available transform to avoid extrapolation)
            transform = self.tf_buffer.lookup_transform(
                "map",
                self.camera_frame,
                rclpy.time.Time(),  # Use latest available transform
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            point_map = do_transform_point(point_stamped, transform)

            self.get_logger().info(f"Transformed camera {point_camera.x:.2f},{point_camera.y:.2f},{point_camera.z:.2f} -> map {point_map.point.x:.2f},{point_map.point.y:.2f},{point_map.point.z:.2f}", throttle_duration_sec=2.0)
            return point_map.point

        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}", throttle_duration_sec=5.0)
            return Point(x=0.0, y=0.0, z=0.0)

    def create_semantic_object(self, detection):
        """Create a single SemanticObject from a YOLO detection"""
        obj = SemanticObject()
        obj.header = self.rgb_image_raw_header
        obj.class_name = self.model.names[int(detection.cls)]
        obj.confidence = float(detection.conf)
        obj.id = int(detection.id) if detection.id is not None else 0

        # Get bounding box center
        box = detection.xyxy[0]  # [x1, y1, x2, y2]
        u = int((box[0] + box[2]) / 2)  # center x
        v = int((box[1] + box[3]) / 2)  # center y

        # Get depth at center and convert to 3D camera frame, then transform to map frame
        if len(self.depth_image_raw) > 0 and v < self.depth_image_raw.shape[0] and u < self.depth_image_raw.shape[1]:
            depth_raw = self.depth_image_raw[v, u]
            depth = float(depth_raw)  # depth is already in meters for float32 encoding
            self.get_logger().info(f"Depth at ({u},{v}): {depth_raw}, fx={self.fx}", throttle_duration_sec=2.0)
            point_camera = self.pixel_to_camera_frame(u, v, depth)
            obj.position = self.transform_to_map_frame(point_camera)
        else:
            obj.position = Point(x=0.0, y=0.0, z=0.0)

        return obj

    def create_detection_array(self, results):
        """Create DetectionArray from YOLO results"""
        detection_array = DetectionArray()
        detection_array.header = self.rgb_image_raw_header

        for detection in results[0].boxes:
            semantic_obj = self.create_semantic_object(detection)
            detection_array.objects.append(semantic_obj)

        return detection_array

    def publish_detection_array(self):
        self.detection_array_pub_.publish(self.detection_array)

    def create_marker_array(self):
        """Create MarkerArray for RViz visualization"""
        marker_array = MarkerArray()

        for i, obj in enumerate(self.detection_array.objects):
            # Create sphere marker for object position
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.rgb_image_raw_header.stamp
            marker.ns = "detections"
            marker.id = i * 2
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = obj.position
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
            marker_array.markers.append(marker)

            # Create text marker for class name and confidence
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.rgb_image_raw_header.stamp
            text_marker.ns = "detections_text"
            text_marker.id = i * 2 + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = obj.position.x
            text_marker.pose.position.y = obj.position.y
            text_marker.pose.position.z = obj.position.z + 0.3
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.15
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"{obj.class_name} ({obj.confidence:.2f})"
            text_marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
            marker_array.markers.append(text_marker)

        return marker_array

    def publish_marker_array(self):
        marker_array = self.create_marker_array()
        self.marker_array_pub_.publish(marker_array)
        
        
    def callback_image_raw(self, msg:Image):    # should get images at 30 hz
        
        # Recieve image and convert from a ros image to a cv2 image. Copy over img header
        rgb_image_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.rgb_image_raw_header = msg.header
        
        # Run YOLO object detection
        results = self.run_detection(rgb_image_raw)

        # Create annotated image and detection array
        self.rgb_image_annotated = self.create_annotated_image(results)
        self.detection_array = self.create_detection_array(results)

        # Publish detection array, markers, and annotated image
        self.publish_detection_array()
        self.publish_marker_array()
        self.publish_image_annotated()

    def callback_depth_raw(self, msg:Image):    # should get images at 30 hz

        # Recieve depth image and convert from a ros image to a cv2 image
        self.depth_image_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def callback_camera_info(self, msg:CameraInfo):

        # Extract camera intrinsics (only once)
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(f"Camera intrinsics: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()