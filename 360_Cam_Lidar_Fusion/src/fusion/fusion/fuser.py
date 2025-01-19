import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2  
from cv_bridge import CvBridge
import cv2
import numpy as np

class SensorFusion(Node):
    def __init__(self):
        super().__init__('image_pub')

        self.subscriber_ = self.create_subscription(
            Image,
            'camera/image',
            self.listener_callback,
            1)

        self.lidar_sub_ = self.create_subscription(
            PointCloud2,
            'lidar_points',
            self.lidar_callback,
            1)
        
        self.publisher_ = self.create_publisher(Image, 'camera/image', 1)

        self.bridge=CvBridge()
        self.latest_image = None
        self.latest_pointcloud = None

        # # Define a transformation matrix for translating 8 inches (0.2032 meters) above
        cos_45 = np.sqrt(2) / 2
        sin_45 = np.sqrt(2) / 2
        # self.transformation_matrix = np.array([
        #     [1, 0, 0, 0],
        #     [0, cos_45, -sin_45, 0.2032],
        #     [0, sin_45, cos_45, 0],
        #     [0, 0, 0, 1]
        # ])

        self.transformation_matrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0.2032],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    def listener_callback(self, msg):
        self.get_logger().info('Received image')

        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_image = cv2.flip(self.latest_image, 1)

        self.process_data()


    def lidar_callback(self, msg):
        self.get_logger().info('Received lidar')

        self.latest_pointcloud = msg

        # points = pc2.read_points(msg, field_names = ("x", "y", "z", "intensity", "ring", "time"), skip_nans=True)
        # for point in points:
        #     x, y, z, intensity, ring, time = point
        #     print(f"x: {x}, y: {y}, z: {z}, intensity: {intensity}, ring: {ring}, time: {time}")
 
    def process_data(self):
        if self.latest_image is not None and self.latest_pointcloud is not None:
            numpy_image = self.latest_image.copy()
            points = pc2.read_points(self.latest_pointcloud, field_names=("x", "y", "z", "intensity", "ring", "time"), skip_nans=True)
            for point in points:
                x, y, z, intensity, ring, time = point
                x, y, z = self.apply_transformation(x, y, z)
                u, v = self.project_point_to_image(x, y, z)
                if 0 <= u < numpy_image.shape[1] and 0 <= v < numpy_image.shape[0]:
                    color = self.get_color_for_distance(distance=np.sqrt(x**2 + y**2 + z**2), ring=ring)
                    cv2.circle(numpy_image, (u, v), 1, color, -1)
            cv2.imshow("Fusion", numpy_image)
            cv2.waitKey(1)

            image_msg = self.bridge.cv2_to_imgmsg(numpy_image, encoding='bgr8')
            self.publisher_.publish(image_msg)

    def apply_transformation(self, x, y, z):
        # Apply transformation matrix to 3D point
        point = np.array([x, y, z, 1])
        # transformed_point = self.transformation_matrix @ point
        transformed_point = np.dot(self.transformation_matrix, point)
        return transformed_point[:3]

    def project_point_to_image(self, x, y, z):
        # Convert 3D point to spherical coordinates
        r = np.sqrt(x**2 + y**2 + z**2)
        theta = np.arctan2(y, x)  # Azimuth angle
        phi = np.arccos(z / r)    # Elevation angle

        # Normalize angles to image coordinates
        u = int((theta + np.pi) / (2 * np.pi) * self.latest_image.shape[1])
        v = int((phi) / np.pi * self.latest_image.shape[0])
        return u, v

    def get_color_for_ring(self, ring):
        # Assign a unique color for each ring
        colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
            (255, 0, 255), (0, 255, 255), (128, 0, 0), (0, 128, 0),
            (0, 0, 128), (128, 128, 0), (128, 0, 128), (0, 128, 128),
            (64, 0, 0), (0, 64, 0), (0, 0, 64), (64, 64, 0)
        ]
        return colors[ring % len(colors)]   

    def get_color_for_distance(self, distance, ring):
        # Map distance to a color gradient (e.g., from blue to red)
        max_distance = 2.0  # Maximum distance for normalization
        normalized_distance = min(distance / max_distance, 1.0)
        blue = int((1 - normalized_distance) * 255)
        red = int(normalized_distance * 255)
        green = ( ring / 16 ) * 255
        return (red, green, blue)

def main(args=None):
    rclpy.init(args=args)
    fuser = SensorFusion()
    rclpy.spin(fuser)
    fuser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
