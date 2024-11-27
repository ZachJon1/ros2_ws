import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv2 import aruco
import math
from dataclasses import dataclass
from typing import List, Tuple, Dict
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy


ARUCO_DICT = {
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

# Dataclass for goal information
@dataclass
class Goal:
    goal_id: int
    position: Tuple[float, float]
    achieved: bool = False
    
@dataclass
class RobotState:
    position: Tuple[float, float] = None
    orientation: float = None
    current_goal: Goal = None

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Initialize variables
        self.bridge = CvBridge()
        self.robot_marker_id = 12
        self.goals: List[Goal] = []
        self.robot_position = None
        self.robot_orientation = None
        
        self.qos_profile = QoSProfile(
            reliability= ReliabilityPolicy.BEST_EFFORT,
            depth= 5
        )
        
        self.declare_parameter('aruco_dict', 'DICT_ARUCO_ORIGINAL')
        aruco_dict = self.get_parameter('aruco_dict').get_parameter_value().string_value
        
        # ArUco dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_dict])
        self.parameters = aruco.DetectorParameters()
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers
        self.create_subscription(Image, '/camera', self.monitor_callback,
                                 qos_profile=self.qos_profile)
        

        # Timer for control loop
        self.create_timer(0.1, self.control_loop)
        

    def monitor_callback(self, msg):
        """Monitor: Process camera feed and detect markers"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            corners, ids, _ = aruco.detectMarkers(cv_image, self.aruco_dict)
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            cv2.imshow('Camera Feed', cv_image)
            cv2.waitKey(1)
            
            if ids is not None:
                self.analyze_markers(corners, ids)
                
        except Exception as e:
            self.get_logger().error(f'Error in monitor_callback: {str(e)}')

    def analyze_markers(self, corners, ids):
        """Analyze: Process marker information"""
        try:
            for i, marker_id in enumerate(ids):
                marker_corners = corners[i][0]
                center = np.mean(marker_corners, axis=0)
                
                if marker_id == self.robot_marker_id:
                    self.robot_position = (center[0], center[1])
                    # Calculate robot orientation
                    self.robot_orientation = math.atan2(
                        marker_corners[1][1] - marker_corners[0][1],
                        marker_corners[1][0] - marker_corners[0][0]
                    )
                else:
                    # Update or add goal position
                    goal_exists = False
                    for goal in self.goals:
                        if goal.goal_id == marker_id:
                            goal_exists = True
                            break
                    
                    if not goal_exists:
                        self.goals.append(Goal(
                            goal_id=marker_id,
                            position=(center[0], center[1])
                        ))
                        
        except Exception as e:
            self.get_logger().error(f'Error in analyze_markers: {str(e)}')

    def plan_path(self) -> Goal:
        """Plan: Determine next goal"""
        if not self.robot_position or not self.goals:
            return None
            
        closest_goal = None
        min_distance = float('inf')
        
        for goal in self.goals:
            if not goal.achieved:
                distance = math.sqrt(
                    (self.robot_position[0] - goal.position[0])**2 +
                    (self.robot_position[1] - goal.position[1])**2
                )
                if distance < min_distance:
                    min_distance = distance
                    closest_goal = goal
        
        return closest_goal

    def control_loop(self):
        """Execute: Control robot movement"""
        if not self.robot_position or not self.robot_orientation:
            return
            
        target_goal = self.plan_path()
        
        if not target_goal:
            # All goals achieved or no goals available
            self.stop_robot()
            return
            
        # Get target position
        target_x, target_y = target_goal.position

        # Calculate distance and angle to the target
        distance = math.hypot(target_x - self.robot_position[0],
                            target_y - self.robot_position[1])
        target_angle = math.atan2(target_y - self.robot_position[1],
                                target_x - self.robot_position[0])
        angle_diff = self.normalize_angle(target_angle - self.robot_orientation)

        # Proportional gain constants
        kp_linear = 0.08
        kp_angular = 0.3

        # Initialize Twist message
        cmd_vel = Twist()

        # Check if goal is achieved
        goal_tolerance = 50
        if distance < goal_tolerance:
            target_goal.achieved = True
            self.stop_robot()
            self.get_logger().info(f'Goal {target_goal.goal_id} achieved!')
            return

        # Calculate velocities with proportional control
        cmd_vel.linear.x = kp_linear * distance
        cmd_vel.angular.z = kp_angular * angle_diff

        # Limit maximum velocities
        max_linear_speed = 0.5
        max_angular_speed = 1.0

        if cmd_vel.linear.x > max_linear_speed:
            cmd_vel.linear.x = max_linear_speed

        if abs(cmd_vel.angular.z) > max_angular_speed:
            cmd_vel.angular.z = max_angular_speed if cmd_vel.angular.z > 0 else -max_angular_speed

        # Publish the command
        self.cmd_vel_pub.publish(cmd_vel)

    def stop_robot(self):
       
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info('Robot stopped')

    @staticmethod
    def normalize_angle(angle):
        
        if angle > math.pi:
            angle -= 2 * math.pi
        return angle

def main():
    rclpy.init()
    controller = RobotController()
    
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(controller)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()