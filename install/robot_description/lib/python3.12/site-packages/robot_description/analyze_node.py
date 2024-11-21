import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math



class Analyze(Node):
    def __init__(self):
        super().__init__('analyze')
        self.create_subscription(Float32MultiArray, '/monitor_data',
                                 self.analyze_callback, 10,
                                 )
        self.analysis_pub = self.create_publisher(Float32MultiArray,
                                                  '/analysis_data', 10)
        self.goals_reached = []
        self.is_obstacle_close = False
        self.closest_goal= None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.goals = []
        self.goals_remaining = []
        
        # scan variables
        self.range_min = 0.08
        self.range_max = 5.0
        self.ll_range = 0.0
        self.fl_range = 0.0
        self.ff_range = 0.0
        self.fr_range = 0.0
        self.rr_range = 0.0
        self.ranges_count = 180
        self.ranges_segments = 5
        self.rays_per_seg = int(self.ranges_count / self.ranges_segments)
        self.scan_dist = 1.0
        self.scan_result = "000000"
        
        self.timer = self.create_timer(1.0, self.publish_analysis_data,)
        
        self.get_logger().info("Starting Analysis...")    

    def analyze_callback(self, msg):
        data = msg.data
        ranges = data[0]
        
        self.rr_range = min(min(ranges[int(0 * self.rays_per_seg):int(1 * self.rays_per_seg)]), self.range_max)
        self.fr_range = min(min(ranges[int(1 * self.rays_per_seg):int(2 * self.rays_per_seg)]), self.range_max)
        self.ff_range = min(min(ranges[int(2 * self.rays_per_seg):int(3 * self.rays_per_seg)]), self.range_max)
        self.fl_range = min(min(ranges[int(3 * self.rays_per_seg):int(4 * self.rays_per_seg)]), self.range_max)
        self.ll_range = min(min(ranges[int(4 * self.rays_per_seg):int(5 * self.rays_per_seg)]), self.range_max)
        rr = ('0' if (self.rr_range > self.scan_dist) else '1')
        fr = ('0' if (self.fr_range > self.scan_dist) else '1')
        ff = ('0' if (self.ff_range > self.scan_dist) else '1')
        fl = ('0' if (self.fl_range > self.scan_dist) else '1')
        ll = ('0' if (self.ll_range > self.scan_dist) else '1')
        fff = ('0' if (self.ff_range > (self.scan_dist / 2.0)) else '1')
        self.scan_result = (fff + ll + fl + ff + fr + rr)
        
        ranges_non_inf = [value for value in ranges if (value != float("inf"))]
        ranges_sum = sum(ranges_non_inf)
        ranges_mean = (ranges_sum / len(ranges_non_inf))
        min_range = min(ranges_non_inf)
              
        self.get_logger().info("LL: %f, FL: %f, FF: %f, FR: %f, RR: %f" % (self.ll_range, self.fl_range, self.ff_range, self.fr_range, self.rr_range))
        self.get_logger().info("Ranges Mean: %f" % (ranges_mean))
        
        self.robot_x = data[1]
        self.robot_y = data[2]
        self.goals = data[3:]

        self.is_obstacle_close = min_range < 0.1
        
        self.get_logger().info("Obstacle Close: %s" % (self.is_obstacle_close))
        
        
        min_distance = float('inf')
        for i in range(0, len(self.goals), 2):
            goal_x = self.goals[i]
            goal_y = self.goals[i+1]
            if (goal_x, goal_y) in self.goals_reached:
                continue
            distance = math.hypot(goal_x - self.robot_x,
                                  goal_y - self.robot_y)
            if distance < 0.1:
                self.goals_reached.append((goal_x, goal_y))
            elif distance < min_distance:
                min_distance = distance
                self.closest_goal = (goal_x, goal_y)

        # remove reached goals from goals list and update goals remaining
        for goal in self.goals_reached:
            if goal in self.goals:
                self.goals.remove(goal)
         
        # go to next goal
        if self.closest_goal:
            self.get_logger().info("Closest Goal: %s" % (self.closest_goal))
        else:
            self.get_logger().info("No Closest Goal")
        
        
    def publish_analysis_data(self):
        analysis = Float32MultiArray()
        analysis.data = [
            int(self.rr_range),
            int(self.fr_range),
            int(self.ff_range),
            int(self.fl_range),
            int(self.ll_range),
            int(self.scan_result),
            int(self.is_obstacle_close),
            self.closest_goal[0] if self.closest_goal else 0.0,
            self.closest_goal[1] if self.closest_goal else 0.0,
            self.robot_x,
            self.robot_y,
        ]
        self.analysis_pub.publish(analysis)
               
def main(args=None):
    rclpy.init(args=args)
    analyze = Analyze()
    rclpy.spin(analyze)
    analyze.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()