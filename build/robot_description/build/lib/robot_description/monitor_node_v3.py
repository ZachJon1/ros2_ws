import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco

ARUCO_DICT = {
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}
class Monitor(Node):
    def __init__(self):
        super().__init__('monitor_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.marker_publisher = self.create_publisher(Int32MultiArray, '/markers', 10)
        self.declare_parameter('aruco_dict', 'DICT_ARUCO_ORIGINAL')
        self.declare_parameter('aruco_marker_size', 0.07)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('aruco_marker_name', 'aruco_marker')
        self.declare_parameter('camera_calibration_parameters_filename', 
                               '/home/azakaria/Documents/sim_v2/ros2_ws/src/robot_description/robot_description/calibration.yml')

        aruco_dict = self.get_parameter('aruco_dict').get_parameter_value().string_value
        self.aruco_marker_size = self.get_parameter('aruco_marker_size').get_parameter_value().double_value
        #image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
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
        self.bridge = CvBridge()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Process the image to detect ArUco markers
        markers = self.detect_aruco_markers(image)
        # Publish the marker positions
        marker_msg = Int32MultiArray(data=markers)
        self.marker_publisher.publish(marker_msg)

    def detect_aruco_markers(self, image_msg):
        poses = []
        # Implement ArUco detection logic here
        gray = cv2.cvtColor(image_msg, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        # Return a list of marker IDs and their positions
        if ids is not None:
            for corner, marker_id in zip(corners, ids.flatten()):
                center = corner[0].mean(axis=0)
                poses.append([int(marker_id), center[0]])
        return poses
            
def main(args=None):
    rclpy.init(args=args)
    monitor = Monitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()