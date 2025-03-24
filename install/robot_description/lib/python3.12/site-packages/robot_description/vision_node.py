import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations
from rclpy.qos import qos_profile_sensor_data


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
        

def main(args=None):
    rclpy.init(args=args)
    vision_node = Vision()
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()