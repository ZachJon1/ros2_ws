import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
from cv2 import aruco
import math
import json
from dataclasses import dataclass, asdict
from typing import List, Tuple, Dict
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor


ARUCO_DICT = {
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

@dataclass
class Goal:
    id: int
    position: Tuple[float, float]
    achieved: bool = False
    assigned_to: str = None

@dataclass
class RobotState:
    id: str
    position: Tuple[float, float] = None
    orientation: float = None
    current_goal: Goal = None

class MultiRobotMAPEController(Node):
    def __init__(self):
        super().__init__('multi_robot_mape_controller')
        
        # Dynamic robot ID parameter
        robot_id_descriptor = ParameterDescriptor(
            description='Unique identifier for the robot'
        )
        self.declare_parameter('robot_id', '12', robot_id_descriptor)
        self.robot_id = self.get_parameter('robot_id').value
        
        # ArUco marker ID matching robot ID
        self.robot_marker_id = int(self.robot_id)
        
        # Logging
        self.get_logger().info(f'Initializing Robot {self.robot_id}')
        
        # QoS Profile for reliable communication
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        # Synchronized state management
        self.robot_states: Dict[str, RobotState] = {}
        self.goals: List[Goal] = []
        
        # Robot's own state
        self.robot_state = RobotState(id=self.robot_id)
        
        # Camera processing
        self.bridge = CvBridge()
        self.declare_parameter('aruco_dict', 'DICT_ARUCO_ORIGINAL')
        aruco_dict = self.get_parameter('aruco_dict').get_parameter_value().string_value
        
        # ArUco dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_dict])
        self.parameters = aruco.DetectorParameters()
        
        # Configurable parameters
        self.declare_parameter('collision_threshold', 50)
        self.collision_threshold = self.get_parameter('collision_threshold').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, f'/robot_{self.robot_id}/cmd_vel', 10)
        
        self.state_pub = self.create_publisher(
            String, '/robot_states', self.qos_profile)
        
        self.goal_pub = self.create_publisher(
            String, '/goal_updates', self.qos_profile)
        
        # Subscribers
        self.create_subscription(
            Image, '/camera', 
            self.monitor_callback, 10)
        
        self.create_subscription(
            String, '/robot_states', 
            self.robot_state_callback, self.qos_profile)
        
        self.create_subscription(
            String, '/goal_updates', 
            self.goal_update_callback, self.qos_profile)
        
        # Control timers
        self.create_timer(0.1, self.control_loop)
        self.create_timer(0.5, self.publish_state)
        
        # Navigation parameters
        self.goal_proximity_threshold = 50
        self.max_linear_velocity = 0.5
        self.max_angular_velocity = 1.0

    def monitor_callback(self, msg):
        """MONITOR: Process camera feed and detect markers"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")            
            # Detect ArUco markers
            corners, ids, _ = aruco.detectMarkers(cv_image, self.aruco_dict)
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            cv2.imshow('Camera Feed', cv_image)
            cv2.waitKey(1)
            
            if ids is not None:
                self.analyze_markers(corners, ids,)
                
        except Exception as e:
            self.get_logger().error(f'Monitor error: {str(e)}')

    def analyze_markers(self, corners, ids, cv_image):
        """ANALYZE: Process marker information"""
        try:
            for i, marker_id in enumerate(ids.flatten()):
                marker_corners = corners[i][0]
                marker_center = np.mean(corners[i][0], axis=0)
                
                # Detect robot marker
                if marker_id == self.robot_marker_id:
                    self.robot_state.position = tuple(marker_center)
                    # Compute orientation from marker corners
                    self.robot_state.orientation = math.atan2(
                        marker_corners[1][1] - marker_corners[0][1],
                        marker_corners[1][0] - marker_corners[0][0]
                    )
                
                # Detect goal markers
                elif marker_id <= 10:
                    # Check if goal already exists
                    goal_exists = any(goal.id == marker_id for goal in self.goals)
                    
                    if not goal_exists:
                        self.goals.append(Goal(
                            id=marker_id,
                            position=tuple(marker_center)
                        ))
                        
        except Exception as e:
            self.get_logger().error(f'Analyze error: {str(e)}')

    def robot_state_callback(self, msg):
        """Handle incoming robot state updates"""
        try:
            state_data = json.loads(msg.data)
            robot_id = state_data.get('id')
            
            # Ignore own state update
            if robot_id != self.robot_id:
                # Update or create robot state
                robot_state = RobotState(
                    id=robot_id,
                    position=tuple(state_data.get('position', (None, None))),
                    orientation=state_data.get('orientation')
                )
                
                if state_data.get('current_goal'):
                    goal_data = state_data['current_goal']
                    robot_state.current_goal = Goal(
                        id=goal_data['id'],
                        position=tuple(goal_data['position']),
                        achieved=goal_data.get('achieved', False),
                        assigned_to=goal_data.get('assigned_to')
                    )
                
                self.robot_states[robot_id] = robot_state
                
        except Exception as e:
            self.get_logger().error(f'State callback error: {str(e)}')

    def goal_update_callback(self, msg):
        """Handle goal updates from other robots"""
        try:
            goal_data = json.loads(msg.data)
            
            for goal in self.goals:
                if goal.id == goal_data['id']:
                    goal.achieved = goal_data.get('achieved', False)
                    goal.assigned_to = goal_data.get('assigned_to')
                    break
                    
        except Exception as e:
            self.get_logger().error(f'Goal update error: {str(e)}')

    def plan_path(self) -> Goal:
        """PLAN: Determine next goal considering robot coordination"""
        if not self.robot_state.position or not self.goals:
            return None
        
        # Check if current goal is still valid
        if (self.robot_state.current_goal and 
            not self.robot_state.current_goal.achieved):
            return self.robot_state.current_goal
        
        # Find unassigned or nearest goal
        best_goal = None
        min_distance = float('inf')
        
        for goal in self.goals:
            # Skip achieved or assigned goals
            if goal.achieved or goal.assigned_to:
                continue
            
            # Calculate distance to goal
            distance = math.sqrt(
                (self.robot_state.position[0] - goal.position[0])**2 +
                (self.robot_state.position[1] - goal.position[1])**2
            )
            
            # Check if other robots are closer
            is_closest = True
            for other_state in self.robot_states.values():
                if other_state.position:
                    other_distance = math.sqrt(
                        (other_state.position[0] - goal.position[0])**2 +
                        (other_state.position[1] - goal.position[1])**2
                    )
                    if other_distance < distance:
                        is_closest = False
                        break
            
            # Update best goal if this is the closest
            if is_closest and distance < min_distance:
                min_distance = distance
                best_goal = goal
        
        # Assign goal if found
        if best_goal:
            best_goal.assigned_to = self.robot_id
            self.robot_state.current_goal = best_goal
            self.publish_goal_update(best_goal)
        
        return best_goal

    def control_loop(self):
        """EXECUTE: Control robot movement"""
        if not self.robot_state.position:
            return
        
        # Check for collision risk
        if self.check_collision_risk():
            self.stop_robot()
            return
        
        # Plan path and get target goal
        target_goal = self.plan_path()
        if not target_goal:
            self.stop_robot()
            return
        
        # Calculate movement parameters
        dx = target_goal.position[0] - self.robot_state.position[0]
        dy = target_goal.position[1] - self.robot_state.position[1]
        
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        # Calculate angle difference
        angle_diff = self.normalize_angle(
            target_angle - self.robot_state.orientation)
        
        # Velocity command
        cmd_vel = Twist()
        
        # Goal proximity check
        if distance < self.goal_proximity_threshold:
            target_goal.achieved = True
            target_goal.assigned_to = None
            self.robot_state.current_goal = None
            self.publish_goal_update(target_goal)
            self.stop_robot()
            self.get_logger().info(f'Goal {target_goal.id} achieved!')
            return
        
        # Navigation control
        if abs(angle_diff) > 0.1:
            # Rotate to face goal
            cmd_vel.angular.z = max(
                min(angle_diff, self.max_angular_velocity), 
                -self.max_angular_velocity
            )
        else:
            # Move towards goal
            cmd_vel.linear.x = min(
                distance * 0.01, 
                self.max_linear_velocity
            )
            cmd_vel.angular.z = angle_diff
        
        self.cmd_vel_pub.publish(cmd_vel)

    def check_collision_risk(self) -> bool:
        """Check potential collisions with other robots"""
        if not self.robot_state.position:
            return False
        
        for other_robot in self.robot_states.values():
            if other_robot.position:
                distance = math.sqrt(
                    (self.robot_state.position[0] - other_robot.position[0])**2 +
                    (self.robot_state.position[1] - other_robot.position[1])**2
                )
                if distance < self.collision_threshold:
                    return True
        return False

    def publish_state(self):
        """Publish robot state for coordination"""
        if not self.robot_state.position:
            return
        
        state_msg = String()
        state_data = {
            'id': self.robot_id,
            'position': self.robot_state.position,
            'orientation': self.robot_state.orientation,
            'current_goal': (asdict(self.robot_state.current_goal) 
                             if self.robot_state.current_goal else None)
        }
        state_msg.data = json.dumps(state_data)
        self.state_pub.publish(state_msg)

    def publish_goal_update(self, goal: Goal):
        """Publish goal status updates"""
        goal_msg = String()
        goal_msg.data = json.dumps({
            'id': goal.id,
            'achieved': goal.achieved,
            'assigned_to': goal.assigned_to
        })
        self.goal_pub.publish(goal_msg)

    def stop_robot(self):
        """Stop robot movement"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    controller = MultiRobotMAPEController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        controller.get_logger().error(f'Controller error: {str(e)}')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()