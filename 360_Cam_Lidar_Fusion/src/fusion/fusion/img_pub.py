import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_pub')

        self.publisher_ = self.create_publisher(Image, 'camera/image', 1)

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.cap = cv2.VideoCapture(2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2880)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().error('Failed to capture image')



def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
