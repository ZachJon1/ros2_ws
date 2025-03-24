import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ObstacleAvoidance(Node):

    def __init__(self):
        # initialize node
        super().__init__("obstacle_avoidance_node")
        self.get_logger().info("Initialized Node: obstacle_avoidance_node")
        
        # initialize cmd_vel publisher
        self.cmd_vel_pub = self.create_publisher(msg_type=Twist,
                                                 topic="/cmd_vel",
                                                 qos_profile=1)
        self.get_logger().info("Initialized Publisher: /cmd_vel")
        
        # initialize twist message variable for cmd_vel
        self.twist_msg = Twist()

        # initialize scan subscriber
        self.scan_sub_qos = QoSProfile(depth=10,
                                       reliability=ReliabilityPolicy.RELIABLE,
                                       durability=DurabilityPolicy.VOLATILE)
        self.scan_sub = self.create_subscription(msg_type=LaserScan,
                                                 topic="/scan",
                                                 callback=self.scan_callback,
                                                 qos_profile=self.scan_sub_qos,
                                                 callback_group=ReentrantCallbackGroup())
        self.get_logger().info("Initialized Subscriber: /scan")
        
        # initialize scan variables
        self.range_min = 0.08   # meters
        self.range_max = 5.00   # meters
        self.ll_range = 0.0   # meters - left range
        self.fl_range = 0.0   # meters - front left range
        self.ff_range = 0.0   # meters - front range
        self.fr_range = 0.0   # meters - front right range
        self.rr_range = 0.0   # meters - right range
        self.ranges_count = 180
        self.ranges_segments = 5
        self.rays_per_seg = int(self.ranges_count / self.ranges_segments)
        self.scan_dist = 1.0   # meters - scan distance tolerance
        self.scan_result = "000000"

        # initialize timer for robot control
        self.timer = self.create_timer(timer_period_sec=1.0,
                                       callback=self.obstacle_avoidance_callback,
                                       callback_group=ReentrantCallbackGroup())
        self.get_logger().info("Initialized Timer")

        # wait for gazebo gui to launch and display the world
        #time.sleep(10.0)

        self.get_logger().info("Starting Obstacle Avoidance...")

        return None

    def __del__(self):
        return None

    def publish_twist_msg(self):
        self.get_logger().info("LinVelX: %f, AngVelZ: %f" % (self.twist_msg.linear.x, self.twist_msg.angular.z))
        self.cmd_vel_pub.publish(self.twist_msg)
        return None

    def scan_callback(self, scan_msg):
        # read the range values from laserscan message
        ranges = scan_msg.ranges

        # calculate range minimum for each segment
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
        fff = ('0' if (self.ff_range > (self.scan_dist / 2.0)) else '1')   # too close in front
        self.scan_result = (fff + ll + fl + ff + fr + rr)
        
        # calculate mean for ranges (exclude inf values)
        ranges_non_inf = [value for value in ranges if (value != float("inf"))]
        ranges_sum = sum(ranges_non_inf)
        ranges_mean = (ranges_sum / len(ranges_non_inf))

        # print the mean range value
        self.get_logger().info("LL: %f, FL: %f, FF: %f, FR: %f, RR: %f" % (self.ll_range, self.fl_range, self.ff_range, self.fr_range, self.rr_range))
        self.get_logger().info("Ranges Mean: %f" % (ranges_mean))
        
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

    def obstacle_avoidance_callback(self):
        # obstacle avoidance algorithm implementation
        scan_case = self.scan_result[2:5]

        # check if robot has obstacle too close in front
        if (self.scan_result[0] == '1'):
            # move backwards
            self.move_robot(value=-0.5)
            time.sleep(1.0)
            # choose a direction to turn
            if (self.ll_range < self.rr_range):
                # turn right
                self.turn_robot(value=-0.5)
            else:
                # otherwise turn left
                self.turn_robot(value=0.5)
            time.sleep(1.0)
        else:
            # otherwise perform usual obstacle cases
            if (scan_case == "000"):
                self.get_logger().info("State: XX-XX-XX")
                self.move_robot(value=0.5)
            elif (scan_case == "001"):
                self.get_logger().info("State: XX-XX-FR")
                self.turn_robot(value=0.5)
            elif (scan_case == "010"):
                self.get_logger().info("State: XX-FF-XX")
                self.turn_robot(value=0.5)
            elif (scan_case == "011"):
                self.get_logger().info("State: XX-FF-FR")
                self.turn_robot(value=0.5)
            elif (scan_case == "100"):
                self.get_logger().info("State: FL-XX-XX")
                self.turn_robot(value=-0.5)
            elif (scan_case == "101"):
                self.get_logger().info("State: FL-XX-FR")
                self.move_robot(value=0.5)
            elif (scan_case == "110"):
                self.get_logger().info("State: FL-FF-XX")
                self.turn_robot(value=-0.5)
            elif (scan_case == "111"):
                self.get_logger().info("State: FL-FF-FR")
                self.turn_robot(value=0.5)
            else:
                self.get_logger().info("State: UNKNOWN")
                self.move_robot(value=0.25)

        return None

def main(args=None):
    # initialize ROS2 communication
    rclpy.init(args=args)
    try:
        # initialize node
        obstacle_avoidance_node = ObstacleAvoidance()
        # initialize executor
        executor = MultiThreadedExecutor(num_threads=6)
        # add node to executor
        executor.add_node(obstacle_avoidance_node)
        try:
            # spin the executor
            executor.spin()
        except Exception as e:
            obstacle_avoidance_node.get_logger().error(f"Exception on executor spin: {e}")
        finally:
            # shutdown executor
            executor.shutdown()
            # destroy node
            obstacle_avoidance_node.get_logger().info("Terminating Obstacle Avoidance...")
            obstacle_avoidance_node.destroy_node()
    except Exception as e:
        print(f"Exception in main: {e}")
    finally:
        # shutdown ROS2 communication
        if rclpy.ok():
            rclpy.shutdown()

    return None

if __name__ == "__main__":
    main()