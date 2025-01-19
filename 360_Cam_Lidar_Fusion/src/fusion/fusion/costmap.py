import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class CostmapNode(Node):
    def __init__(self):
        super().__init__('costmap_node')

        # Create a publisher for the costmap
        self.publisher_ = self.create_publisher(OccupancyGrid, 'costmap', 10)

        # Create a timer to publish the costmap at a regular interval
        self.timer = self.create_timer(1.0, self.publish_costmap)

        # Initialize the costmap
        self.width = 200
        self.height = 200
        self.resolution = 0.1
        self.origin_x = -10.0
        self.origin_y = -10.0
        self.costmap = np.zeros((self.height, self.width), dtype=np.int8)
        self.costmap[100, 100] = 100

        # Example: Set some obstacles in the costmap
        self.costmap[20:30, 20:30] = 100  # Set a square obstacle

    def publish_costmap(self):
        # Convert the costmap to an OccupancyGrid message
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = "map"
        occupancy_grid.info.resolution = self.resolution
        occupancy_grid.info.width = self.width
        occupancy_grid.info.height = self.height
        occupancy_grid.info.origin.position.x = self.origin_x
        occupancy_grid.info.origin.position.y = self.origin_y
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0

        # Flatten the costmap and assign it to the data field
        occupancy_grid.data = self.costmap.flatten().tolist()

        # Publish the OccupancyGrid message
        self.publisher_.publish(occupancy_grid)

def main(args=None):
    rclpy.init(args=args)
    costmap_node = CostmapNode()
    rclpy.spin(costmap_node)
    costmap_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()