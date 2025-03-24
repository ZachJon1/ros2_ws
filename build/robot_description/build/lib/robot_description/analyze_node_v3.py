import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import math

class Analyze(Node):
    def __init__(self):
        super().__init__('analyze_node')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/markers',
            self.marker_callback,
            10)
        self.goal_publisher = self.create_publisher(Float32MultiArray, '/next_goal', 10)
        self.achieved_goals = set()
        
        self.robot_pose_publisher = self.create_publisher(
            Float32MultiArray,
            '/robot_pose',
            10                  
        )
        self.robot_pose = None

    def marker_callback(self, msg):
        markers = msg.data
        robot_pose, goal_poses = self.parse_markers(markers)
        self.robot_pose_publisher.publish(robot_pose)
        next_goal = self.select_next_goal(robot_pose, goal_poses)
        if next_goal:
            self.publish_next_goal(next_goal)
    
    def parse_markers(self, markers):
        # Parse markers to separate robot and goal positions
        robot_pose = None
        goal_poses = []
        for marker in markers:
            for marker_id, pose in markers:
                if marker_id == 12:
                    robot_pose = pose
                else:
                    goal_poses.append(pose)
                    
        return robot_pose, goal_poses

    def select_next_goal(self, robot_pose, goal_poses):
        # Select the closest unachieved goal
        closest_goal = None
        closest_distance = float('inf')
        for goal in goal_poses:
            distance = self.calculate_distance(robot_pose, goal)
            if distance < closest_distance and goal not in self.achieved_goals:
                closest_goal = goal
                closest_distance = distance
        if closest_goal:
            self.achieved_goals.add(closest_goal)
            return closest_goal
        return None
    
    def calculate_distance(self, pose1, pose2):
        return math.sqrt((pose1[0] - pose2[0])**2 + (pose1[1] - pose2[1])**2)

    def publish_next_goal(self, goal):
        goal_msg = Float32MultiArray(data=goal)
        self.goal_publisher.publish(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    analyze = Analyze()
    rclpy.spin(analyze)
    analyze.destroy_node()
    rclpy.shutdown()