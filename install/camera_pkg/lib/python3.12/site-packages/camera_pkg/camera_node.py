import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Camera device index 
        self.camera_index = 2
        
        # Create publisher for camera image
        self.publisher = self.create_publisher(Image, '/camera', 10)
        
        # Timer to publish frames
        self.timer = self.create_timer(0.1, self.publish_frame)
        
        # OpenCV camera capture
        self.cap = cv2.VideoCapture(self.camera_index)
        
        # CvBridge for converting between ROS and OpenCV images
        self.bridge = CvBridge()

    def publish_frame(self):
        # Capture frame from camera
        ret, frame = self.cap.read()
        
        if ret:
            # Convert OpenCV image to ROS image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            
            # Publish the image
            self.publisher.publish(ros_image)

    def __del__(self):
        # Release camera on shutdown
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()