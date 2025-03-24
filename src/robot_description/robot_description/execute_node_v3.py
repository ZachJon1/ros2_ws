import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class Execute(Node):
    def __init__(self):
        super().__init__('execute_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/plan',
            self.command_callback,
            10)
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

    def command_callback(self, msg):
        # Send commands to robot's motors
        self.execute_command(msg)

    def execute_command(self, command):
        #motor control logic here
        twist = Twist()
        twist.linear.x = command[0]
        twist.angular.z = command[1]
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"Executing command: {command}")

def main(args=None):
    rclpy.init(args=args)
    execute = Execute()
    rclpy.spin(execute)
    execute.destroy_node()
    rclpy.shutdown()