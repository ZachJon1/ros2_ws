import rclpy
import rclpy.executors
from rclpy.node import Node
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



ARUCO_DICT = {
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}

class Vision(Node):

    def __init__(self):
        super().__init__('vision')
        
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
            qos_profile=qos_profile_sensor_data
        )
        
        #Publishers
        self.robot_poses_pub = self.create_publisher(
            PoseArray,
            '/robot_poses',
            10
        )
        
        self.goal_poses_pub = self.create_publisher(
            PoseArray,
            '/goal_poses',
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
            
# Monitor node 
class Monitor(Node):
    
    def __init__(self):
        super().__init__('monitor')
        
        self.robot_markers ={12: 'robot_1', 13: 'robot_2',
                             14: 'robot_3', 15: 'robot_4'}
        
        self.get_logger().info("[INFO] monitor node has been started")
        
        # Subscribers - Monitor loop subscribes to sensors, robot and goal poses
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal',
            self.goal_callback,
            10
        )
        
        self.robot_poses_sub = self.create_subscription(
            PoseArray,
            '/robot_poses',
            self.robot_poses_callback,
            10
        )
        
        self.goal_poses_sub = self.create_subscription(
            PoseArray,
            '/goal_poses',
            self.goal_poses_callback,
            10
        )
        
        # Publishers - Monitor loop publishes robot and goal poses
        
        self.sensor_data_pub = self.create_publisher(
            Float32MultiArray,
            '/sensor_data',
            10
        )
        self.current_pose = None
        self.goal_poses = None
        self.obstacle_distances = None
        self.scan_result = '000000'
        self.range_max = 2.00
        self.range_min = 0.02
        self.rays_per_seg = 25
        self.scan_dist = 1.0
        self.twist_msg = Twist()
        
        
        self.get_logger().info("[INF] Initialized Publisher: /sensor_data")    
        # Robot Id to be monitored
        self.robot_id = 'robot_1'
    
    def laser_callback(self, data):
        self.scan_data = data.ranges

        
    def odom_callback(self, data):
        self.current_pose = data.pose.pose
        self.publish_sensor_data()
        
    def goal_callback(self, data):
        self.goal_poses = data.pose
        self.publish_sensor_data()
        
    def robot_poses_callback(self, data):
        robot_index = list(self.robot_markers.values()).index(self.robot_id)
        if robot_index < len(data.poses):
            self.current_pose = data.poses[robot_index]
            self.publish_sensor_data()
        
    def goal_poses_callback(self, data):
        if len(data.poses) > 0:
            self.goal_poses = data.poses[0]
            self.publish_sensor_data()
            
    def publish_sensor_data(self):
        if all([self.current_pose, self.goal_poses, self.obstacle_distances is not None]):
            sensor_data = Float32MultiArray()
            data.data = self.prepare_sensor_data()
            self.sensor_data_pub.publish(sensor_data)
        
    def prepare_sensor_data(self):
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        goal_x = self.goal_poses.position.x
        goal_y = self.goal_poses.position.y
        min_obstacle_distance = np.min(self.obstacle_distances)
        return [current_x, current_y, goal_x, goal_y, min_obstacle_distance]
        
class Analyze(Node):
    
    def __init__(self):
        super().__init__('Analyze')
        
        # Subscribe
        self.sensor_data_sub = self.create_subscription(
            Float32MultiArray,
            'sensor_data',
            self.analyze_callback,
            10
        )
        
        # Publish
        self.analysis_pub = self.create_publisher(
            Float32MultiArray,
            'analysis',
            10
        )
        
        self.get_logger().info("[INFO] analyze node has been started")
        
    def analyze_callback(self, msg):
        self.get_logger().info("[INFO] analyze callback has been called")
        # get sensor data
        curr_x, curr_y, goal_x, goal_y, min_obstacle_distance = msg.data
        
        # calculate distance to goal
        distance_to_goal = np.sqrt((goal_x - curr_x)**2 + (goal_y - curr_y)**2)
        angle_to_goal = np.arctan2(goal_y - curr_y, goal_x - curr_x)
        
        # flag to indicate if the robot is facing the goal
        facing_goal = False
        if abs(angle_to_goal) < np.pi/4:
            facing_goal = True
            
        # flag to indicate if the robot is close to the goal
        close_to_goal = False
        if distance_to_goal < 0.5:
            close_to_goal = True
            
        # flag to determine obstacle avoidance
        is_obstacle_close = min_obstacle_distance < 0.5
        
        # Publish analysis
        analysis_msg = Float32MultiArray()
        analysis_msg.data = [distance_to_goal, angle_to_goal, facing_goal,
                             close_to_goal, float(is_obstacle_close)]
        self.analysis_pub.publish(analysis_msg)
        
class Planner(Node):
    
    def __init__(self):
        super().__init__('planner')
        
        # Subscribe 
        self.anlaysis_sub = self.create_subscription(
            Float32MultiArray,
            'analysis',
            self.plan_action_callback,
            10    
        )
        
        # Publish
        self.plan_pub = self.create_publisher(
            Float32MultiArray,
            'plan',
            10
        )
        
    def plan_action_callback(self, msg):
        distance_to_goal, angle_to_goal, facing_goal, close_to_goal, is_obstacle_close = msg.data
        
        if is_obstacle_close:
            linear_vel = 0.0
            angular_vel = 0.5
        else:
            linear_vel = min(0.5, distance_to_goal)
            angular_vel = angle_to_goal
            
        plan = Float32MultiArray()
        plan.data = [linear_vel, angular_vel]
        self.plan_pub.publish(plan)
        

class Executor(Node):
    
    def __init__(self):
        super().__init__('Executor')
        
        # Subscribe
        self.plan_sub = self.create_subscription(
            Float32MultiArray,
            'plan',
            self.execute_plan_callback,
            10
        )
        
        # Publish
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
    def execute_plan_callback(self, msg):
        linear_vel, angular_vel = msg.data
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd_vel)
        
        
def main(args=None):
    
    rclpy.init(args=args)
    vision = Vision()
    monitor = Monitor()
    analyzer = Analyze()
    planner = Planner()
    executor_node = Executor()
    
    # spin all nodes 
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=6)
    executor.add_node(vision)
    executor.add_node(monitor)
    executor.add_node(analyzer)
    executor.add_node(planner)
    executor.add_node(executor_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        rclpy.shutdown()
        
        
if __name__ == '__main__':
    main()
        
        

        
        
        