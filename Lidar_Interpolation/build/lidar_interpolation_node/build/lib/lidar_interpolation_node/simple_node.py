import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2  # For working with PointCloud2 data
import numpy as np  # For interpolation calculations

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')  # Node name
        self.get_logger().info('SimpleNode is up and running!')

        # Declare parameters
        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/lidar_points')
        self.declare_parameter('interpolation_multiple', 1.0)

        # Get parameter values
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.interpolation_multiple = int(self.get_parameter('interpolation_multiple').value)

        # Log parameters
        self.get_logger().info(f"Subscribing to: {input_topic}")
        self.get_logger().info(f"Publishing to: {output_topic}")
        self.get_logger().info(f"Interpolation multiple: {self.interpolation_multiple}")

        # Subscriber and publisher
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.velodyne_callback,
            10  # QoS depth
        )
        self.publisher = self.create_publisher(PointCloud2, output_topic, 10)

    def velodyne_callback(self, msg):
        # Read all fields from the point cloud
        points = list(pc2.read_points(
            msg, field_names=('x', 'y', 'z', 'intensity', 'ring', 'time'), skip_nans=True
        ))

        # Group points by ring
        from collections import defaultdict
        ring_groups = defaultdict(list)
        for point in points:
            ring_groups[int(point[4])].append(point)  # Group by `ring`

        # Prepare a list for all interpolated points
        # TODO: Define size to avoid reallocation        
        all_points = []

        # Iterate through each ring group
        ring_keys = list(ring_groups.keys())
        for i in range(len(ring_keys) - 1):
            # Get the ring points
            ring1_points = ring_groups[ring_keys[i]]
            ring2_points = ring_groups[ring_keys[i + 1]]

            # Size of the rings
            ring1_size = len(ring1_points)
            ring2_size = len(ring2_points)

            # Add original points from ring1
            all_points.extend(ring1_points)

            # Interpolate between the two rings
            for j in range(0, ring1_size):
                for k in range(0, self.interpolation_multiple):                
                    t = k / self.interpolation_multiple

                    # Floor of k / ring1_size * ring2_size
                    m = int(j / ring1_size * ring2_size)

                    # ensure m is within bounds
                    m = min(m, ring2_size - 1)

                    interpolated_point = (
                        ring1_points[j][0] * (1 - t) + ring2_points[m][0] * t,  # Interpolate x
                        ring1_points[j][1] * (1 - t) + ring2_points[m][1] * t,  # Interpolate y
                        ring1_points[j][2] * (1 - t) + ring2_points[m][2] * t,  # Interpolate z
                        ring1_points[j][3] * (1 - t) + ring2_points[m][3] * t,  # Interpolate intensity
                        ring1_points[j][4],  # Use the ring of the first point
                        ring1_points[j][5] * (1 - t) + ring2_points[m][5] * t   # Interpolate time
                    )
                    all_points.append(interpolated_point)
        # Add the last ring
        all_points.extend(ring_groups[ring_keys[-1]])

        # Create a new PointCloud2 message with the original and interpolated points
        modified_msg = pc2.create_cloud(msg.header, msg.fields, all_points)

        # Publish the modified PointCloud2 data
        self.publisher.publish(modified_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
