import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class WhiteColorDetector(Node):
    def __init__(self):
        super().__init__('white_color_detector')
        self.subscription = self.create_subscription(
            Image, 'camera/image', self.listener_callback, 10)
        
        self.publisher = self.create_publisher(Image, 'camera/white_image', 10)
        self.bridge = CvBridge()
        
    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            lower_white = np.array([35, 60, 70])
            upper_white = np.array([85, 255, 255])
            
            mask = cv2.inRange(hsv_image, lower_white, upper_white)
            
            white_detected = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            
            white_image_msg = self.bridge.cv2_to_imgmsg(white_detected, encoding = 'bgr8')
            
            self.publisher.publish(white_image_msg)
            
            cv2.imshow('White Color Detection', white_detected)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    white_color_detector = WhiteColorDetector()
    rclpy.spin(white_color_detector)
    white_color_detector.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()