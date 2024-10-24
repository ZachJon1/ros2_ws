import rclpy 
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

ARUCO_DICT = {
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
}

class ArUcoDetectionPoseEstimation(Node):
    
    def __init__(self):
        super().__init__('aruco_detection_pose_estimation')
        
        self.declare_parameter('aruco_dict', 'DICT_ARUCO_ORIGINAL')
        self.declare_parameter('aruco_marker_size', 0.03)
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

        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.listener_callback,
            qos_profile=qos_profile_sensor_data
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.bridge = CvBridge()
        
    def listener_callback(self, data):
        
        self.get_logger().info('Received image')
        
        cur_frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(
            cur_frame, 
            self.this_aruco_dict,                                                                 
            )
        
        if marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(cur_frame, corners, marker_ids)
            
            rvecs, tvecs, object_points = cv2.aruco.estimatePoseSingleMarkers(
                corners, 
                self.aruco_marker_size, self.mtx, self.dst)
        
            for i, marker_id in enumerate(marker_ids):  
 
                # Create the coordinate transform
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'camera_depth_frame'
                t.child_frame_id = self.aruco_marker_name
            
                # Store the translation (i.e. position) information
                t.transform.translation.x = tvecs[i][0][0]
                t.transform.translation.y = tvecs[i][0][1]
                t.transform.translation.z = tvecs[i][0][2]
        
                # Store the rotation information
                rotation_matrix = np.eye(4)
                rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                r = R.from_matrix(rotation_matrix[0:3, 0:3])
                quat = r.as_quat()   
                
                # Quaternion format     
                t.transform.rotation.x = quat[0] 
                t.transform.rotation.y = quat[1] 
                t.transform.rotation.z = quat[2] 
                t.transform.rotation.w = quat[3] 
        
                # Send the transform
                self.tf_broadcaster.sendTransform(t)    
                        
                # Draw the axes on the marker
                cv2.aruco.drawDetectedMarkers(cur_frame, corners, marker_ids)       
            
        cv2.imshow('aruco_detection_pose_estimation', cur_frame)
        cv2.waitKey(1)
        
def main(args=None):
    
    rclpy.init(args=args)
    
    aruco_node = ArUcoDetectionPoseEstimation()
    
    rclpy.spin(aruco_node)
    aruco_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()