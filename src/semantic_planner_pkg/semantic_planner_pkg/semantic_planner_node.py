#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vehicle_interfaces.msg import DetectionArray, SemanticObject
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import math

class SemanticPlannerNode(Node):
    def __init__(self):
        super().__init__("semantic_planner_node")

        self.detection_array_sub_ = self.create_subscription(
            DetectionArray, "/detection_array", self.callback_detection_array, 10)
        self.navigation_command_sub_ = self.create_subscription(
            String, "/navigation_command", self.callback_navigation_command, 10)

        self.semantic_map_pub_ = self.create_publisher(MarkerArray, "semantic_map", 10)
        self.goal_pose_pub_ = self.create_publisher(PoseStamped, "/goal_pose", 10)

        # Semantic map: list of SemanticObject
        self.semantic_map = []

        # TF2 for getting robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Distance threshold for considering objects as duplicates (in meters)
        self.declare_parameter('duplicate_distance_threshold', 1.0)
        self.duplicate_threshold = self.get_parameter('duplicate_distance_threshold').value

        # Minimum confidence threshold to add detection to semantic map
        self.declare_parameter('min_confidence_threshold', 0.7)
        self.min_confidence = self.get_parameter('min_confidence_threshold').value

        self.get_logger().info(f"Semantic Planner Node started with duplicate threshold: {self.duplicate_threshold}m, min confidence: {self.min_confidence}")

    def calculate_distance(self, pos1, pos2):
        """Calculate Euclidean distance between two positions"""
        dx = pos1.x - pos2.x
        dy = pos1.y - pos2.y
        dz = pos1.z - pos2.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def find_duplicate_index(self, new_obj):
        """Find if a similar object exists in the semantic map"""
        for i, existing_obj in enumerate(self.semantic_map):
            # Check if same class and within distance threshold
            if existing_obj.class_name == new_obj.class_name:
                distance = self.calculate_distance(existing_obj.position, new_obj.position)
                if distance < self.duplicate_threshold:
                    return i
        return -1

    def update_semantic_map(self, detection_array):
        """Update semantic map with new detections, filtering duplicates"""
        for new_obj in detection_array.objects:
            # Skip invalid detections (zero position indicates failure)
            if new_obj.position.x == 0.0 and new_obj.position.y == 0.0 and new_obj.position.z == 0.0:
                continue

            # Skip detections below minimum confidence threshold
            if new_obj.confidence < self.min_confidence:
                continue

            duplicate_idx = self.find_duplicate_index(new_obj)

            if duplicate_idx >= 0:
                # Duplicate found - replace if new confidence is higher
                if new_obj.confidence > self.semantic_map[duplicate_idx].confidence:
                    self.get_logger().info(
                        f"Updating {new_obj.class_name} at ({new_obj.position.x:.2f}, {new_obj.position.y:.2f}) "
                        f"with higher confidence: {new_obj.confidence:.2f} > {self.semantic_map[duplicate_idx].confidence:.2f}",
                        throttle_duration_sec=1.0
                    )
                    self.semantic_map[duplicate_idx] = new_obj
            else:
                # New unique object - add to map
                self.semantic_map.append(new_obj)
                self.get_logger().info(
                    f"Added {new_obj.class_name} at ({new_obj.position.x:.2f}, {new_obj.position.y:.2f}, {new_obj.position.z:.2f}) "
                    f"with confidence {new_obj.confidence:.2f}",
                    throttle_duration_sec=1.0
                )

    def create_semantic_map_markers(self):
        """Create MarkerArray for semantic map visualization"""
        marker_array = MarkerArray()

        for i, obj in enumerate(self.semantic_map):
            # Create sphere marker for object position
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "semantic_map"
            marker.id = i * 2
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = obj.position
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.9
            marker_array.markers.append(marker)

            # Create text marker for class name and confidence
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "semantic_map_text"
            text_marker.id = i * 2 + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = obj.position.x
            text_marker.pose.position.y = obj.position.y
            text_marker.pose.position.z = obj.position.z + 0.4
            text_marker.pose.orientation.x = 0.0
            text_marker.pose.orientation.y = 0.0
            text_marker.pose.orientation.z = 0.0
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.2
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"{obj.class_name} ({obj.confidence:.2f})"
            marker_array.markers.append(text_marker)

        return marker_array

    def publish_semantic_map(self):
        """Publish the semantic map as markers"""
        marker_array = self.create_semantic_map_markers()
        self.semantic_map_pub_.publish(marker_array)

    def callback_detection_array(self, msg: DetectionArray):
        """Process incoming detection array and update semantic map"""
        self.update_semantic_map(msg)
        self.publish_semantic_map()

    def get_robot_position(self):
        """Get current robot position in map frame"""
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            return transform.transform.translation
        except Exception as e:
            self.get_logger().warn(f"Failed to get robot position: {e}", throttle_duration_sec=5.0)
            return None

    def find_closest_object_in_map(self, target_name):
        """Find closest object in semantic map by name (case-insensitive partial match)"""
        target_name = target_name.lower().strip()
        robot_pos = self.get_robot_position()

        if robot_pos is None:
            self.get_logger().warn("Cannot find closest object without robot position")
            return None

        closest_obj = None
        min_distance = float('inf')

        for obj in self.semantic_map:
            if target_name in obj.class_name.lower():
                distance = self.calculate_distance(robot_pos, obj.position)
                if distance < min_distance:
                    min_distance = distance
                    closest_obj = obj

        return closest_obj

    def navigate_to_object(self, target_name):
        """Send navigation goal to closest matching object location"""
        obj = self.find_closest_object_in_map(target_name)

        if obj is None:
            self.get_logger().warn(f"Object '{target_name}' not found in semantic map")
            return

        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position = obj.position
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        self.goal_pose_pub_.publish(goal_pose)
        self.get_logger().info(
            f"Navigating to {obj.class_name} at ({obj.position.x:.2f}, {obj.position.y:.2f}, {obj.position.z:.2f})"
        )

    def callback_navigation_command(self, msg: String):
        """Process navigation commands like 'go to chair'"""
        cmd = msg.data.lower().strip()

        if "go to" in cmd:
            target = cmd.replace("go to", "").strip()
            self.navigate_to_object(target)
        else:
            self.get_logger().warn(f"Unknown command format: '{msg.data}'")

def main(args=None):
    rclpy.init(args=args)
    node = SemanticPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
