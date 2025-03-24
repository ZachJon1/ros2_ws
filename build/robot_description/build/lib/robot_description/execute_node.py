import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import time
import math

class Execute(Node):
    def __init__(self):
        super().__init__('execute')
        self.create_subscription(Float32MultiArray, '/plan_data',
                                 self.execute_callback, 10,
                                 )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist_msg = Twist()
        self.get_logger().info("Execute Node Initialized")
    
    def publish_twist_msg(self):
        self.cmd_vel_pub.publish(self.twist_msg)
        return None
        
    def move_robot(self, value):
        # front +ve, back -ve
        self.twist_msg.linear.x = value
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = 0.0
        self.publish_twist_msg()
        return None
    
    def turn_robot(self, value):
        # left +ve, right -ve
        self.twist_msg.linear.x = 0.0
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = value
        self.publish_twist_msg()
        return None

    def stop_robot(self):
        self.twist_msg.linear.x = 0.0
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = 0.0
        self.publish_twist_msg()
        return None
    
    def execute_callback(self, msg):
        data = msg.data
        # data = [ float(self.rr_range),
                # float(self.fr_range),
                # float(self.ff_range),
                # float(self.fl_range),
                # float(self.ll_range),
                # float(self.scan_result),
                # float(is_obstacle_close),
                # closest_goal[0] if closest_goal else 0.0,
                # closest_goal[1] if closest_goal else 0.0,
                # robot_x,
                # robot_y,
                # float(goals_remaining),]
                      
        rr_range = data[0]
        ll_range = data[4]
        scan_result = data[5]

        closest_goal_x = data[7]
        closest_goal_y = data[8]
        robot_x = data[9]
        robot_y = data[10]
        mode = data[11]
        
        # plan logic
        scan_result = str(scan_result)
        scan_case = scan_result[2:5]

        if mode == 1:
            # Obstacle Avoidance
            if scan_result[0] == '1':
                self.move_robot(value=-0.5)
                time.sleep(1.0)
                
                if ll_range < rr_range:
                    self.move_robot(value=-0.5)
                else:
                    self.move_robot(value=0.5)
                time.sleep(1.0)
            else:
                if  (scan_case == "000"):
                    self.move_robot(value=0.5)
                elif (scan_case == "001"):
                    self.turn_robot(value=0.5)
                elif (scan_case == "010"):
                    self.turn_robot(value=0.5)
                elif (scan_case == "011"):
                    self.turn_robot(value=0.5)
                elif (scan_case == "100"):
                    self.turn_robot(value=0.5)
                elif (scan_case == "101"):
                    self.move_robot(value=0.5)
                elif (scan_case == "110"):
                    self.turn_robot(value=0.5)
                elif (scan_case == "111"):
                    self.turn_robot(value=0.5)
                else:
                    self.move_robot(value=0.25)
                    
        elif mode == 0:
            # Goal Navigation
                
            # navigate to closest goal         
            closest_goal = (closest_goal_x, closest_goal_y)                    
            if closest_goal:
                goal_x = closest_goal[0]
                goal_y = closest_goal[1]
                
                x_diff = goal_x - robot_x
                y_diff = goal_y - robot_y
                
                angle_to_goal = math.atan2(y_diff, x_diff)
                
                angle_to_goal = math.degrees(angle_to_goal)
                
                if angle_to_goal > 180:
                    angle_to_goal -= 360
                
                if angle_to_goal < -180:
                    angle_to_goal += 360
                
                if angle_to_goal > 45:
                    self.turn_robot(value=-0.5)
                elif angle_to_goal < -45:
                    self.turn_robot(value=0.5)
                else:
                    self.move_robot(value=0.5)
                    
                # # update goals
                # for goal in goals_remaining:
                #     if goal[0] == closest_goal_x and goal[1] == closest_goal_y:
                #         goals_remaining.remove(goal)
                    
                # # all goals reached 
                # if len(goals_remaining) == 0:
                #     self.stop_robot()
                #     goals_remaining = 0
            else:
                self.stop_robot()
        
        
def main(args=None):
    rclpy.init(args=args)
    execute = Execute()
    rclpy.spin(execute)
    execute.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()