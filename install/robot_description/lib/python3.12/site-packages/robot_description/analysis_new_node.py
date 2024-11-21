import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math

class Analyze(Node):
    def __init__(self):
        super().__init__('analyze_node')
        self.create_subscription(
            Float32MultiArray,
            '/monitor_data',
            self.analyze_callback,
            10)
        self.analysis_pub = self.create_publisher(
            Float32MultiArray,
            '/analysis_data',
            10)
        self.goals_reached = set()

    def analyze_callback(self, msg):
        data = msg.data
        ranges = data[:360]
        robot_x = data[360]
        robot_y = data[361]
        goal_positions = [(data[i], data[i+1]) for i in range(362, len(data), 2)]

        is_obstacle_close = any(r < 0.2 for r in ranges if r > 0.0)
        closest_goal = None
        min_distance = float('inf')

        for i, goal_pos in enumerate(goal_positions):
            if i in self.goals_reached:
                continue
            distance = math.hypot(goal_pos[0] - robot_x, goal_pos[1] - robot_y)
            if distance < 0.2:
                self.goals_reached.add(i)
            elif distance < min_distance:
                min_distance = distance
                closest_goal = goal_pos

        analysis = Float32MultiArray()
        if closest_goal:
            analysis.data = [float(is_obstacle_close), closest_goal[0], closest_goal[1], robot_x, robot_y]
        else:
            analysis.data = [float(is_obstacle_close), robot_x, robot_y]
        self.analysis_pub.publish(analysis)

def main(args=None):
    rclpy.init(args=args)
    node = Analyze()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()