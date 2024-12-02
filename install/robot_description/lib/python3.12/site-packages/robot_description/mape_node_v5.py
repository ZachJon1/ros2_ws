import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from cv2 import aruco
import math
from dataclasses import dataclass, field
from typing import List, Tuple, Dict
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf_transformations import euler_from_quaternion
import csv
import os

ARUCO_DICT = {
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

@dataclass
class Goal:
    goal_id: int
    position: Tuple[float, float]
    achieved: bool = False

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Initialize variables
        self.bridge = CvBridge()
        self.robot_marker_id = 12
        
        # Camera Calibration Parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # ArUco Marker Size (in meters)
        self.marker_size = 0.06  
        
        # Goal Management
        self.all_goals: List[Goal] = []  # Saves all discovered goals
        self.unvisited_goals: List[Goal] = []  # Goals yet to be visited
        self.current_goal: Goal = None  # Current goal being pursued
        
        # Robot State
        self.robot_position = None
        self.robot_orientation = 0.0  # Yaw in radians
        self.imu_orientation = 0.0
        
        # Movement State
        self.movement_stage = 'ALIGN'  # Can be 'ALIGN', 'MOVE', or 'STOP'
        
        # QoS Profile
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Declare parameters
        self.declare_parameter('aruco_dict', 'DICT_ARUCO_ORIGINAL')
        aruco_dict = self.get_parameter('aruco_dict').get_parameter_value().string_value
        
        # ArUco dictionary and parameters with subpixel refinement
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_dict])
        
        # Configure detector parameters with subpixel corner refinement
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.parameters.cornerRefinementWinSize = 11
        self.parameters.cornerRefinementMaxIterations = 30
        self.parameters.cornerRefinementMinAccuracy = 0.1
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers
        self.create_subscription(Image, '/camera', self.monitor_callback,
                                 qos_profile=self.qos_profile)
        self.create_subscription(Imu, '/imu', self.imu_callback, 
                                 qos_profile=self.qos_profile)
        self.camera_info_subscriber = self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback,
                                 qos_profile=self.qos_profile)
        
        self.goals_csv_path = os.path.join(
            os.path.expanduser('~'), 'goals.csv'
        )
        
        # Timer for control loop
        self.create_timer(0.3, self.control_loop)

    def camera_info_callback(self, msg):
        """Receive camera calibration parameters"""
        # Convert camera_info to numpy arrays
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        
        # Unsubscribe after receiving calibration to avoid repeated updates
        self.destroy_subscription(self.camera_info_subscriber)
        # print calibration parameters
        self.get_logger().info(f'Camera matrix: {self.camera_matrix}')
        self.get_logger().info(f'Distortion coefficients: {self.dist_coeffs}')
        self.get_logger().info('Camera calibration received')

    def monitor_callback(self, msg):
        """Monitor: Process camera feed and detect markers"""
        try:
            # Skip processing if camera calibration is not yet received
            if self.camera_matrix is None or self.dist_coeffs is None:
                return

            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect markers
            corners, ids, _ = cv2.aruco.detectMarkers(
                cv_image, 
                self.aruco_dict, 
                parameters=self.parameters
            )
            
            if ids is not None:
                # Estimate pose for each marker
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, 
                    self.marker_size,  # Marker size in meters 
                    self.camera_matrix, 
                    self.dist_coeffs
                )
                
                # Draw detected markers and axes
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                
                # Draw axis for each detected marker
                for i, marker_id in enumerate(ids.flatten()):
                    cv2.drawFrameAxes(
                        cv_image, 
                        self.camera_matrix, 
                        self.dist_coeffs, 
                        rvecs[i], 
                        tvecs[i], 
                        self.marker_size
                    )
                    
                    # Optional: Log marker pose information
                    self.get_logger().info(
                        f"Marker {marker_id} - Translation: {tvecs[i].flatten()}, "
                        f"Rotation (rodrigues): {rvecs[i].flatten()}"
                    )
                
                cv2.imshow('Camera Feed with Marker Pose', cv_image)
                cv2.waitKey(1)
                
                # Analyze markers as before
                self.analyze_markers(corners, ids, tvecs)
                
        except Exception as e:
            self.get_logger().error(f'Error in monitor_callback: {str(e)}')

    def analyze_markers(self, corners, ids, tvecs=None):
        """Analyze: Process marker information and manage goals"""
        try:
            for i, marker_id in enumerate(ids.flatten()):
                marker_corners = corners[i][0]
                center = np.mean(marker_corners, axis=0)
                
                if marker_id == self.robot_marker_id:
                    # Update robot position from ArUco marker
                    self.robot_position = (center[0], center[1])
                    
                    if tvecs is not None:
                        pass
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
    
    def imu_callback(self, msg):
        """Process IMU data to get robot orientation"""
        # Convert quaternion to euler angles
        quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        _, _, self.imu_orientation = euler_from_quaternion(quaternion)

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

    def calculate_goal_angle(self, target_x, target_y):
        """Calculate angle to the goal using IMU orientation"""
        # Calculate desired angle to the goal
        goal_angle = math.atan2(
            target_y - self.robot_position[1],
            target_x - self.robot_position[0]
        )
        
        # Normalize both angles
        goal_angle = self.normalize_angle(goal_angle)
        current_angle = self.normalize_angle(self.imu_orientation)
        
        # Calculate the smallest rotation needed
        angle_diff = self.normalize_angle(goal_angle - current_angle)
        
        return angle_diff

    def control_loop(self):
        """Execute: Control robot movement"""
        if not self.robot_position:
            return
        
        # If no current goal, select the closest unvisited goal
        if not self.current_goal and self.unvisited_goals:
            self.current_goal = self.select_closest_goal()
            # Reset movement stage
            self.movement_stage = 'ALIGN'
            self.get_logger().info(
                f'Next goal to pursue: Goal ID {self.current_goal.goal_id}, '
                f'Position {self.current_goal.position}'
            )
        
        # If no goals available or all goals visited
        if not self.current_goal:
            self.stop_robot()
            return
            
        # Get target position
        target_x, target_y = self.current_goal.position

        # Calculate distance and angle to the target
        distance = math.hypot(target_x - self.robot_position[0],
                            target_y - self.robot_position[1])
        
        # Calculate angle difference using IMU
        angle_diff = self.calculate_goal_angle(target_x, target_y)

        # Proportional gain constants
        kp_angular = 0.5
        kp_linear = 0.1

        # Initialize Twist message
        cmd_vel = Twist()

        # Check if goal is achieved
        goal_tolerance = 50  # Pixel tolerance
        if distance < goal_tolerance:
            # Mark current goal as achieved and remove from unvisited goals
            self.current_goal.achieved = True
            self.unvisited_goals.remove(self.current_goal)
            
            self.get_logger().info(f'Goal {self.current_goal.goal_id} achieved!')
            
            # Clear current goal to trigger goal selection in next iteration
            self.current_goal = None
            self.movement_stage = 'ALIGN'
            
            self.stop_robot()
            return

        # Two-stage movement
        if self.movement_stage == 'ALIGN':
            # First, rotate to face the goal using IMU
            cmd_vel.angular.z = kp_angular * angle_diff
            
            # If angle is close enough, switch to move stage
            if abs(angle_diff) < 0.1:  
                self.movement_stage = 'MOVE'
                self.get_logger().info('Aligned with goal. Moving forward.')
        
        elif self.movement_stage == 'MOVE':
            # Move straight towards the goal
            cmd_vel.linear.x = kp_linear * distance
            
            # If angle deviates too much, go back to alignment
            if abs(angle_diff) > 0.2:  
                self.movement_stage = 'ALIGN'
                self.get_logger().info('Realigning with goal.')

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
        controller.save_goals_to_csv()
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()