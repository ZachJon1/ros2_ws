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
        self.create_subscription(Float32MultiArray, '/analysis_data',
                                 self.plan_callback, 10)
        self.plan_pub = self.create_publisher(Float32MultiArray,
                                              '/plan_data', 10)
        
    # Decide on either goal or obstacle avoidance mode
    
    def plan_callback(self, msg):
        data = msg.data

        rr_range = data[0]
        fr_range = data[1]
        ff_range = data[2]
        fl_range = data[3]
        ll_range = data[4]
        scan_result = data[5]
        is_obstacle_close = data[6]
        closest_goal_x = data[7]
        closest_goal_y = data[8]
        robot_x = data[9]
        robot_y = data[10]

        
        plan = Float32MultiArray()

        # select goal or obstacle
        mode = 1 if is_obstacle_close else 0
        
        plan.data = [
            rr_range,
            fr_range,
            ff_range,
            fl_range,
            ll_range,
            float(scan_result),
            is_obstacle_close,
            closest_goal_x,
            closest_goal_y,
            robot_x,
            robot_y,
            mode
        ]
        
        self.plan_pub.publish(plan)

        self.get_logger().info("Planner Node Initialized")

        
    # def plan_callback(self, msg):
    #     data = msg.data
    #     # data = [ float(self.rr_range),
    #             # float(self.fr_range),
    #             # float(self.ff_range),
    #             # float(self.fl_range),
    #             # float(self.ll_range),
    #             # float(self.scan_result),
    #             # float(is_obstacle_close),
    #             # closest_goal[0] if closest_goal else 0.0,
    #             # closest_goal[1] if closest_goal else 0.0,
    #             # robot_x,
    #             # robot_y,
    #             # float(goals_remaining),]
                      
    #     rr_range = data[0]
    #     fr_range = data[1]
    #     ff_range = data[2]
    #     fl_range = data[3]
    #     ll_range = data[4]
    #     scan_result = data[5]
    #     is_obstacle_close = data[6]
    #     closest_goal_x = data[7]
    #     closest_goal_y = data[8]
    #     robot_x = data[9]
    #     robot_y = data[10]
    #     goals_remaining = data[11]
        
    #     # plan logic
    #     scan_case = scan_result[2:5]
        
    #     plan = Float32MultiArray()
        
    #     if scan_result[0] == '1':
    #         self.move_robot(value=-0.5)
    #         time.sleep(1.0)
            
    #         if ll_range < rr_range:
    #             self.move_robot(value=-0.5)
    #         else:
    #             self.move_robot(value=0.5)
    #         time.sleep(1.0)
    #     else:
    #         if  (scan_case == "000"):
    #             self.move_robot(value=0.5)
    #         elif (scan_case == "001"):
    #             self.turn_robot(value=0.5)
    #         elif (scan_case == "010"):
    #             self.turn_robot(value=0.5)
    #         elif (scan_case == "011"):
    #             self.turn_robot(value=0.5)
    #         elif (scan_case == "100"):
    #             self.turn_robot(value=0.5)
    #         elif (scan_case == "101"):
    #             self.move_robot(value=0.5)
    #         elif (scan_case == "110"):
    #             self.eturn_robot(value=0.5)
    #         elif (scan_case == "111"):
    #             self.turn_robot(value=0.5)
    #         else:
    #             self.move_robot(value=0.25)
                
    #     # Goal Navigation
    #     closest_goal = None
    #     min_distance = float('inf')
    #     for goal in goals_remaining:
    #         distance = math.sqrt((goal[0] - robot_x)**2 + (goal[1] - robot_y)**2)
    #         if distance < min_distance:
    #             min_distance = distance
    #             closest_goal = goal
    #         # # remove current_goal 
    #         # if goal[0] == closest_goal_x and goal[1] == closest_goal_y:
    #         #     goals_remaining.remove(goal)
                
    #     if closest_goal:
    #         goal_x = closest_goal[0]
    #         goal_y = closest_goal[1]
            
    #         x_diff = goal_x - robot_x
    #         y_diff = goal_y - robot_y
            
    #         angle_to_goal = math.atan2(y_diff, x_diff)
            
    #         angle_to_goal = math.degrees(angle_to_goal)
            
    #         if angle_to_goal > 180:
    #             angle_to_goal -= 360
            
    #         if angle_to_goal < -180:
    #             angle_to_goal += 360
            
    #         if angle_to_goal > 45:
    #             self.turn_robot(value=-0.5)
    #         elif angle_to_goal < -45:
    #             self.turn_robot(value=0.5)
    #         else:
    #             self.move_robot(value=0.5)
    #     else:
    #         self.stop_robot()
    #         goals_remaining = 0

    #     plan.data = [linear_vel, angular_vel]
    #     self.plan_pub.publish(plan)
def main(args=None):
    rclpy.init(args=args)
    planner = Planner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
