import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
import numpy as np

ARUCO_DICT = {
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

class MarkerDetector(Node):
    def __init__(self):
        super().__init__('marker_detector')
        
        # Subscribers and Publishers
        self.image_subscription = self.create_subscription(
            Image, '/camera', self.detect_markers, 10
        )
        
        self.marker_publisher = self.create_publisher(
            Point, '/marker_positions', 20
        )
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # ArUco dictionary
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = aruco.DetectorParameters()
        
    def detect_markers(self, msg):
        # Convert ROS image to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Detect markers
        corners, ids, _ = aruco.detectMarkers(
            cv_image, 
            self.aruco_dict, 
            parameters=self.parameters
        )
        
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                # Calculate marker center
                marker_center = np.mean(corners[i][0], axis=0)
                
                # Publish marker position
                point_msg = Point()
                point_msg.x = float(marker_center[0])
                point_msg.y = float(marker_center[1])
                point_msg.z = float(marker_id)  # Marker ID as z coordinate
                
                self.marker_publisher.publish(point_msg)

def main(args=None):
    rclpy.init(args=args)
    marker_detector = MarkerDetector()
    
    try:
        rclpy.spin(marker_detector)
    except KeyboardInterrupt:
        pass
    finally:
        marker_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
