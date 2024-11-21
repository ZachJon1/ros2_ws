import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class Plan(Node):
    def __init__(self):
        super().__init__('plan_node')
        self.create_subscription(
            Float32MultiArray,
            '/analysis_data',
            self.plan_callback,
            10)
        self.plan_pub = self.create_publisher(
            Float32MultiArray,
            '/plan_data',
            10)

    def plan_callback(self, msg):
        data = msg.data
        is_obstacle_close = bool(data[0])
        plan = Float32MultiArray()

        if is_obstacle_close:
            plan.data = [1.0]  
        elif len(data) > 3:
            goal_x = data[1]
            goal_y = data[2]
            robot_x = data[3]
            robot_y = data[4]
            plan.data = [0.0, goal_x, goal_y, robot_x, robot_y]
        else:
            plan.data = [2.0]  

        self.plan_pub.publish(plan)

def main(args=None):
    rclpy.init(args=args)
    node = Plan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()