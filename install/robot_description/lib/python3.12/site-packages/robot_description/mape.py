import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Float32MultiArray
from rclpy.executors import MultiThreadedExecutor
import math
from rclpy.qos import qos_profile_sensor_data
import cv2 
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from rclpy.callback_groups import ReentrantCallbackGroup

ARUCO_DICT = {
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}

class Vision(Node):
    
    def __init__(self):
        super().__init__('vision')
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.declare_parameter('aruco_dict', 'DICT_ARUCO_ORIGINAL')
        self.declare_parameter('aruco_marker_size', 0.07)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('aruco_marker_name', 'aruco_marker')
        self.declare_parameter('camera_calibration_parameters_filename', '/home/azakaria/Documents/sim_v2/ros2_ws/src/robot_description/robot_description/calibration.yml')

        aruco_dict = self.get_parameter('aruco_dict').get_parameter_value().string_value
        self.aruco_marker_size = self.get_parameter('aruco_marker_size').get_parameter_value().double_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.aruco_marker_name = self.get_parameter('aruco_marker_name').get_parameter_value().string_value
        self.camera_calibration_parameters_filename = self.get_parameter('camera_calibration_parameters_filename').get_parameter_value().string_value
        
        self.get_logger().info("[INFO] aruco dict '%s'" % aruco_dict)
        self.this_aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_dict])
        self.this_aruco_parameters = cv2.aruco.DetectorParameters()
        
        cv2_file = cv2.FileStorage(
            self.camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ
        )
        self.mtx = cv2_file.getNode("K").mat()
        self.dst = cv2_file.getNode("D").mat()
        cv2_file.release()
        
        self.get_logger().info("[INFO] camera calibration parameters have been loaded")
        
        self.get_logger().info("[INFO] detecting markers {}".format(aruco_dict))
        self.this_aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_dict])
        self.this_aruco_parameters = cv2.aruco.DetectorParameters()
        
        # Image Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.callback_group
        )
        
        #Publishers
        self.robot_poses_pub = self.create_publisher(
            PoseArray,
            'robot_poses',
            10
        )
        
        self.goal_poses_pub = self.create_publisher(
            PoseArray,
            'goal_poses',
            10
        )
        
        # Transform Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Bridge
        self.bridge = CvBridge()
        
        # Diction to store maker ids 
        self.robot_markers ={12: 'robot_1', 13: 'robot_2',
                             14: 'robot_3', 15: 'robot_4'}
        self.goal_markers ={1: 'goal_1', 2: 'goal_2', 
                            3: 'goal_3', 4: 'goal_4', 
                            5: 'goal_5', 6: 'goal_6', 
                            7: 'goal_7', 8: 'goal_8',
                            9: 'goal_9', 10: 'goal_10',}
        
    def image_callback(self, data):
        
        # Convert Image to OpenCV format
        cur_frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        
        # Detect Aruco Markers
        (corners, ids, rejected) = cv2.aruco.detectMarkers(
            cur_frame,
            self.this_aruco_dict,
        )
        
        if ids is not None:
            
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(cur_frame, corners, ids)
            
            # Get rotation and translation vectors
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.aruco_marker_size,
                self.mtx,
                self.dst
            )
            
            robot_poses = PoseArray()
            goal_poses = PoseArray()
            
            # robot and goal poses
            robot_poses.header.frame_id = 'camera'
            goal_poses.header.frame_id = 'camera'
            robot_poses.header.stamp = self.get_clock().now().to_msg()
            goal_poses.header.stamp = self.get_clock().now().to_msg()
            
            for i in range(len(ids)):
                
                marker_id = ids[i][0]
                
                pose = Pose()
                rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
                quatenion = tf_transformations.quaternion_from_matrix(
                    np.vstack((np.hstack((rotation_matrix, tvecs[i].T)),
                               [0, 0, 0, 1]))
                ) # 4x4 matrix - homogeneous coordinates
                
                pose.position.x = tvecs[i][0][0]
                pose.position.y = tvecs[i][0][1]
                pose.position.z = tvecs[i][0][2]
                pose.orientation.x = quatenion[0]
                pose.orientation.y = quatenion[1]
                pose.orientation.z = quatenion[2]
                pose.orientation.w = quatenion[3]
                
                # Broadcast transform
                if marker_id in self.robot_markers:
                    robot_poses.poses.append(pose)
                    self.broadcast_transform(pose,
                                             self.robot_markers[marker_id],
                                             'camera')
                    
                elif marker_id in self.goal_markers:
                    goal_poses.poses.append(pose)
                    self.broadcast_transform(pose,
                                             self.goal_markers[marker_id],
                                             'camera')
                    
            # Publish robot and goal poses
            self.robot_poses_pub.publish(robot_poses)
            self.goal_poses_pub.publish(goal_poses)
        cv2.imshow('Marker Detection', cur_frame)
        cv2.waitKey(1)
            
    def broadcast_transform(self, pose, child_frame, parent_frame):
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        
        self.tf_broadcaster.sendTransform(t)

class Monitor(Node):
    def __init__(self):
        super().__init__('monitor')
        self.scan_data = None
        self.robot_pose = None
        self.goal_poses = []
        self.callback_group_monitor = ReentrantCallbackGroup()
        self.callback_group_robot = ReentrantCallbackGroup()
        self.callback_group_goal = ReentrantCallbackGroup()

        self.create_subscription(LaserScan, '/scan', 
                                 self.laser_scan_callback, 10,
                                 callback_group=self.callback_group_monitor)
        self.create_subscription(PoseArray, '/robot_poses', 
                                 self.robot_pose_callback, 10,
                                 callback_group=self.callback_group_robot)
        self.create_subscription(PoseArray, '/goal_poses', 
                                 self.goal_poses_callback, 10,
                                 callback_group=self.callback_group_goal)

        self.monitor_pub = self.create_publisher(Float32MultiArray, 
                                                 '/monitor_data', 10)
        self.timer = self.create_timer(0.1, self.publish_monitor_data)

    def laser_scan_callback(self, msg):
        self.scan_data = msg.ranges

    def robot_pose_callback(self, msg):
        if msg.poses:
            self.robot_pose = msg.poses[0]

    def goal_poses_callback(self, msg):
        self.goal_poses = msg.poses

    def publish_monitor_data(self):
        if self.scan_data and self.robot_pose and self.goal_poses:
            data = Float32MultiArray()
            data.data = [self.scan_data, self.robot_pose.position.x, self.robot_pose.position.y]
            for goal in self.goal_poses:
                data.data.extend([goal.position.x, goal.position.y])
            self.monitor_pub.publish(data)

class Analyze(Node):
    def __init__(self):
        super().__init__('analyze')
        self.callback_group_analyze = ReentrantCallbackGroup()
        self.create_subscription(Float32MultiArray, '/monitor_data', self.analyze_callback, 10,
                                 callback_group=self.callback_group_analyze)
        self.analysis_pub = self.create_publisher(Float32MultiArray, '/analysis_data', 10)
        self.goals_reached = []
        
        # scan variables
        self.range_min = 0.08
        self.range_max = 5.0
        self.ll_range = 0.0
        self.fl_range = 0.0
        self.ff_range = 0.0
        self.fr_range = 0.0
        self.rr_range = 0.0
        self.ranges_count = 120
        self.ranges_segments = 5
        self.rays_per_seg = int(self.ranges_count / self.ranges_segments)
        self.scan_dist = 1.0
        self.scan_result = "00000"
    

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
        
        robot_x = data[1]
        robot_y = data[2]
        goals = data[3:]

        is_obstacle_close = min_range < 0.5

        closest_goal = None
        min_distance = float('inf')
        for i in range(0, len(goals), 2):
            goal_x = goals[i]
            goal_y = goals[i+1]
            if (goal_x, goal_y) in self.goals_reached:
                continue
            distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
            if distance < 0.2:
                self.goals_reached.append((goal_x, goal_y))
            elif distance < min_distance:
                min_distance = distance
                closest_goal = (goal_x, goal_y)

        goals_remaining = bool(closest_goal)

        analysis = Float32MultiArray()
        analysis.data = [
            float(is_obstacle_close),
            closest_goal[0] if closest_goal else 0.0,
            closest_goal[1] if closest_goal else 0.0,
            robot_x,
            robot_y,
            float(goals_remaining)
        ]
        self.analysis_pub.publish(analysis)

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

    vision = Vision()
    monitor = Monitor()
    analyze = Analyze()
    planner = Planner()
    execute = Execute()

    executor = MultiThreadedExecutor(8)

    executor.add_node(vision)
    executor.add_node(monitor)
    executor.add_node(analyze)
    executor.add_node(planner)
    executor.add_node(execute)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        vision.destroy_node()
        monitor.destroy_node()
        analyze.destroy_node()
        planner.destroy_node()
        execute.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()