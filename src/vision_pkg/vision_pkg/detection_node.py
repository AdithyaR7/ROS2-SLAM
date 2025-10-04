#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header

from ultralytics import YOLO
import numpy as np
 
class DetectionNode(Node):
    def __init__(self):
        super().__init__("detection_node")
        
        self.rgb_video_sub_ = self.create_subscription(
            Image, "/camera/camera/image_raw", self.callback_image_raw, 10)
        
        self.image_annotated_pub_ = self.create_publisher(Image, "image_annotated", 10) 

        # Initialize params
        self.bridge = CvBridge()            # bridge to convert ros image to cv2 image
        self.rgb_image_annotated = []             # initialize rgb image from Image msg
        self.rgb_image_raw_header = Header()    # initialize header from Image msg 

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
        annotated = results[0].plot()           # fast drawing of bounding boxes
        return annotated
        
    def callback_image_raw(self, msg:Image):    # should get images at 30 hz
        
        # Recieve image and convert from a ros image to a cv2 image. Copy over img header
        rgb_image_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.rgb_image_raw_header = msg.header
        
        # Run YOLO object detection and get annotated frame
        self.rgb_image_annotated = self.run_detection(rgb_image_raw)
        
        self.publish_image_annotated()
        
def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()