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
from dataclasses import dataclass, field
from typing import List, Tuple, Dict
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy
import os
import csv

ARUCO_DICT = {
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

@dataclass
class Goal:
    goal_id: int
    position: Tuple[float, float]
    achieved: bool = False

@dataclass
class RobotState:
    position: Tuple[float, float] = None
    orientation: float = None

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Initialize variables
        self.bridge = CvBridge()
        self.robot_marker_id = 12
        
        # Goal Management
        self.all_goals: List[Goal] = []  # Saves all discovered goals
        self.unvisited_goals: List[Goal] = []  # Goals yet to be visited
        self.current_goal: Goal = None  # Current goal being pursued
        
        # Robot State
        self.robot_position = None
        self.robot_orientation = None
        
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=5
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
        
        self.goals_csv_path = os.path.join(
            os.path.expanduser('~'), 'goals.csv'
        )
        
        # Timer for control loop
        self.create_timer(5, self.control_loop)

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
        """Analyze: Process marker information and manage goals"""
        try:
            for i, marker_id in enumerate(ids.flatten()):
                marker_corners = corners[i][0]
                center = np.mean(marker_corners, axis=0)
                
                if marker_id == self.robot_marker_id:
                    # Update robot position and orientation
                    self.robot_position = (center[0], center[1])
                    self.robot_orientation = math.atan2(
                        marker_corners[1][1] - marker_corners[0][1],
                        marker_corners[1][0] - marker_corners[0][0]
                    )
            
                else:
                    # Check if goal is already known
                    goal_exists = any(goal.goal_id == marker_id for goal in self.all_goals)
                    
                    if not goal_exists:
                        # Create new goal and add to lists
                        new_goal = Goal(
                            goal_id=marker_id,
                            position=(center[0], center[1])
                        )
                        self.all_goals.append(new_goal)
                        self.unvisited_goals.append(new_goal)
                        self.get_logger().info(f'New goal discovered: {marker_id}')
                        
        except Exception as e:
            self.get_logger().error(f'Error in analyze_markers: {str(e)}')

    def select_closest_goal(self) -> Goal:
        """Select the closest unvisited goal"""
        if not self.robot_position or not self.unvisited_goals:
            return None
            
        closest_goal = min(
            self.unvisited_goals, 
            key=lambda goal: math.hypot(
                self.robot_position[0] - goal.position[0],
                self.robot_position[1] - goal.position[1]
            )
        )
        
        return closest_goal

    def control_loop(self):
        """Execute: Control robot movement"""
        if not self.robot_position or not self.robot_orientation:
            return
        
        # If no current goal, select the closest unvisited goal
        if not self.current_goal and self.unvisited_goals:
            self.current_goal = self.select_closest_goal()
            self.get_logger().info(f'Pursuing goal {self.current_goal.goal_id}')
            
            self.get_logger().info(f'Next goal: {self.current_goal.goal_id}')
            self.get_logger().info(f'Next goal position: {self.current_goal.position}')
            self.get_logger().info(f'Robot position: {self.robot_position}')
        
        # If no goals available or all goals visited
        if not self.current_goal:
            self.stop_robot()
            return
            
        # Get target position
        target_x, target_y = self.current_goal.position

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
            # Mark current goal as achieved and remove from unvisited goals
            self.current_goal.achieved = True
            self.unvisited_goals.remove(self.current_goal)
            
            self.get_logger().info(f'Goal {self.current_goal.goal_id} achieved!')
            
            # Clear current goal to trigger goal selection in next iteration
            self.current_goal = None
            
            self.stop_robot()
            return

        # Calculate velocities with proportional control
        cmd_vel.linear.x = kp_linear * distance
        cmd_vel.angular.z = kp_angular * angle_diff

        # Limit maximum velocities
        max_linear_speed = 0.5
        max_angular_speed = 1.0

        cmd_vel.linear.x = min(cmd_vel.linear.x, max_linear_speed)
        cmd_vel.angular.z = max(min(cmd_vel.angular.z, max_angular_speed), -max_angular_speed)

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
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def save_goals_to_csv(self):
        """Save discovered goal locations to a CSV file"""
        try:
            # Ensure all goals have been processed
            goals_to_save = self.all_goals

            # Open the CSV file in write mode
            with open(self.goals_csv_path, 'w', newline='') as csvfile:
                # Create CSV writer
                csvwriter = csv.writer(csvfile)
                
                # Write header
                csvwriter.writerow(['Goal ID', 'X Position', 'Y Position', 'Achieved'])
                
                # Write goal data
                for goal in goals_to_save:
                    csvwriter.writerow([
                        goal.goal_id, 
                        goal.position[0], 
                        goal.position[1], 
                        goal.achieved
                    ])
            
            self.get_logger().info(f'Goals saved to {self.goals_csv_path}')
        
        except Exception as e:
            self.get_logger().error(f'Error saving goals to CSV: {str(e)}')

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