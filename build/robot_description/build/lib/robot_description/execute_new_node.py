import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math

class Execute(Node):
    def __init__(self):
        super().__init__('execute_node')
        self.create_subscription(
            Float32MultiArray,
            '/plan_data',
            self.execute_callback,
            10)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

    def execute_callback(self, msg):
        data = msg.data
        command = data[0]
        twist = Twist()

        if command == 1.0:
            twist.angular.z = 0.5  
        elif command == 0.0:
            goal_x = data[1]
            goal_y = data[2]
            robot_x = data[3]
            robot_y = data[4]
            angle_to_goal = math.atan2(goal_y - robot_y, goal_x - robot_x)
            twist.linear.x = 0.2
            twist.angular.z = angle_to_goal
        elif command == 2.0:
            twist.linear.x = 0.0  
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = Execute()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()