import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import Twist
import math

class Planner(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/next_goal',
            self.goal_callback,
            10)
        
        self.robot_subscription = self.create_subscription(
            Float32MultiArray,
            '/robot_pose',
            self.robot_callback,
            10
        )
        self.command_publisher = self.create_publisher(Float32MultiArray, '/plan', 10)

    def goal_callback(self, msg):
        goal = msg.data
        self.goal = goal
        
    def robot_callback(self, msg):
        robot_pose = msg.data
        commands = self.compute_commands(robot_pose, self.goal)
        self.publish_commands(commands)

    def compute_commands(self, robot_pose, goal):
        # Generate motion commands to reach the goal position
        
        # compute distance and angle to goal
        distance = self.calculate_distance(robot_pose, goal)
        angle = self.calculate_angle(robot_pose, goal)

        # compute linear and angular velocity
        linear_velocity = self.compute_linear_velocity(distance)
        angular_velocity = self.compute_angular_velocity(angle)

        # create Twist message
        commands = Twist()
        commands.linear.x = linear_velocity
        commands.angular.z = angular_velocity
        return commands       

    def calculate_distance(self, pose1, pose2):
        # Calculate the Euclidean distance between two poses
        return ((pose1.position.x - pose2.position.x)**2 + (pose1.position.y - pose2.position.y)**2)**0.5

    def calculate_angle(self, pose1, pose2):
        # Calculate the angle between two poses
        return math.atan2(pose2.position.y - pose1.position.y, pose2.position.x - pose1.position.x)

    def compute_linear_velocity(self, distance):
        # Compute the linear velocity based on the distance to the goal
        return 0.2 * distance

    def compute_angular_velocity(self, angle):
        # Compute the angular velocity based on the angle to the goal
        return 0.1 * angle

    def publish_commands(self, commands):
        self.command_publisher.publish(commands)

def main(args=None):
    rclpy.init(args=args)
    planner = Planner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()