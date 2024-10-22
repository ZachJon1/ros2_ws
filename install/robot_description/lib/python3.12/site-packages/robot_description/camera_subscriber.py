import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.listener_callback,
            10)
        self.subscription  
        self.get_logger().info('Camera Subscriber has been started')
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.camera_info_callback,
            10
        )
        self.camera_info_sub
        self.get_logger().info('Camera Info Subscriber has been started')
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Receiving image')
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow('Camera Image', cv_image)
        cv2.waitKey(1)
        
    def camera_info_callback(self, msg):
        self.get_logger().info('Receiving camera info')
        print(msg)
        self.get_logger().info('Camera info has been received')

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()