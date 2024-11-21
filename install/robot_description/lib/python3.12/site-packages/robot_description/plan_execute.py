import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from rclpy.executors import MultiThreadedExecutor
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class Planner(Node):
    def __init__(self):
        super().__init__('planner')
        self.create_subscription(Float32MultiArray, '/analysis_data', self.plan_callback, 10)
        self.plan_pub = self.create_publisher(Float32MultiArray, '/plan_data', 10)

    def plan_callback(self, msg):
        data = msg.data
        is_obstacle = bool(data[0])
        goal_x = data[1]
        goal_y = data[2]
        robot_x = data[3]
        robot_y = data[4]
        goals_remaining = bool(data[5])

        plan = Float32MultiArray()
        if not goals_remaining:
            linear_vel = 0.0
            angular_vel = 0.0
        elif is_obstacle:
            linear_vel = 0.0
            angular_vel = 0.5
        else:
            angle_to_goal = math.atan2(goal_y - robot_y, goal_x - robot_x)
            linear_vel = 0.2
            angular_vel = angle_to_goal
        plan.data = [linear_vel, angular_vel]
        self.plan_pub.publish(plan)

class Execute(Node):
    def __init__(self):
        super().__init__('execute')
        self.create_subscription(Float32MultiArray, '/plan_data', self.execute_callback, 10,
                                 )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def execute_callback(self, msg):
        data = msg.data
        linear_vel = data[0]
        angular_vel = data[1]

        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist)
        
def main(args=None):
    rclpy.init(args=args)
    planner = Planner()
    execute = Execute()

    executor = MultiThreadedExecutor()
    executor.add_node(planner)
    executor.add_node(execute)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
