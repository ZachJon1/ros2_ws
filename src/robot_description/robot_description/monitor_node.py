import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32MultiArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class Monitor(Node):
    def __init__(self):
        super().__init__('monitor')
        self.scan_data = None
        self.robot_pose = None
        self.goal_poses = []
        self.callback_group_monitor = ReentrantCallbackGroup()
        self.callback_group_robot = ReentrantCallbackGroup()
        self.callback_group_goal = ReentrantCallbackGroup()
        
        self.scan_sub_qos = QoSProfile(depth=10,
                                       reliability=ReliabilityPolicy.RELIABLE,
                                       durability=DurabilityPolicy.VOLATILE)

        self.scan_sub = self.create_subscription(LaserScan, '/scan', 
                                 self.laser_scan_callback,
                                 callback_group=self.callback_group_monitor,
                                 qos_profile=self.scan_sub_qos)
        self.create_subscription(PoseArray, '/robot_poses', 
                                 self.robot_pose_callback, 10,
                                 callback_group=self.callback_group_robot)
        self.create_subscription(PoseArray, '/goal_poses', 
                                 self.goal_poses_callback, 10,
                                 callback_group=self.callback_group_goal)

        self.monitor_pub = self.create_publisher(Float32MultiArray, 
                                                 '/monitor_data', 10)
        self.timer = self.create_timer(1, self.publish_monitor_data)
        self.get_logger().info("Monitor Node Initialized")

    def laser_scan_callback(self, msg):
        self.scan_data = msg.ranges
        self.get_logger().info("Laser Scan Data Received")
        

    def robot_pose_callback(self, msg):
        if msg.poses:
            self.robot_pose = msg.poses[0]
            self.get_logger().info("Robot Pose Received")

    def goal_poses_callback(self, msg):
        self.goal_poses = msg.poses
        self.get_logger().info("Goal Poses Received")

    def publish_monitor_data(self):
        self.get_logger().info("Monitor Data")
        self.get_logger().info(str(self.scan_data))
        self.get_logger().info(str(self.robot_pose))
        self.get_logger().info(str(self.goal_poses))
        if  self.scan_data and self.robot_pose and self.goal_poses:
            data = Float32MultiArray()
            data.data = list(self.scan_data) + [self.robot_pose.position.x,
                                                self.robot_pose.position.y]
            for goal in self.goal_poses:
                data.data.extend([goal.position.x, goal.position.y])
            
            self.get_logger().info(str(data.data))
            self.monitor_pub.publish(data)    

def main(args=None):
    rclpy.init(args=args)
    monitor = Monitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
