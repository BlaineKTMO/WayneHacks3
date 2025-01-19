import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid 
from map_msgs.msg import OccupancyGridUpdate
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import numpy as np
import cupy as cp
import sensor_msgs_py.point_cloud2 as pc2
import cv2

class CostmapNode(Node):
    def __init__(self):
        super().__init__('costmap_node')

        # Create a publisher for the costmap
        self.publisher_ = self.create_publisher(OccupancyGrid, 'costmap', 10)
        self.update_publisher_ = self.create_publisher(OccupancyGridUpdate, 'costmap_update', 10)

        # Create a timer to publish the costmap at a regular interval
        self.timer = self.create_timer(1.0, self.publish_costmap)

        # Initialize the costmap
        self.width = 200
        self.height = 200
        self.resolution = 0.1
        self.origin_x = -10.0
        self.origin_y = -10.0
        self.costmap = cp.zeros((self.height, self.width), dtype=cp.int8)

        # Subscribe to the LiDAR point cloud topic
        self.lidar_sub = self.create_subscription(
            PointCloud2, 'velodyne_points', self.lidar_callback, 10)

        # Subscribe to the lane segmented image topic
        self.image_sub = self.create_subscription(
            Image, 'camera/white_image', self.image_callback, 10)

        self.bridge = CvBridge()
        self.latest_image = None

        # Define a transformation matrix for rotating 45 degrees about the y-axis
        cos_45 = cp.sqrt(2) / 2
        sin_45 = cp.sqrt(2) / 2
        self.transformation_matrix = cp.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, -0.203],
            [0, 0, 0, 1]
        ])
        
        self.previous_updated_cells = set()

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def lidar_callback(self, msg):
        if self.latest_image is None:
            return

        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_list = list(points)
        points_array = np.array(points_list, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])

        x = cp.array(points_array['x'])
        y = cp.array(points_array['y'])
        z = cp.array(points_array['z'])

        ones = cp.ones_like(x)
        points_matrix = cp.stack((x, y, z, ones), axis=-1)
        transformed_points = cp.dot(points_matrix, self.transformation_matrix.T)
        x, y, z = transformed_points[:, 0], transformed_points[:, 1], transformed_points[:, 2]

        r = cp.sqrt(x**2 + y**2 + z**2)
        theta = cp.arctan2(y, x)
        phi = cp.arccos(z / r)

        u = ((theta + cp.pi) / (2 * cp.pi) * self.latest_image.shape[1]).astype(int)
        v = ((phi) / cp.pi * self.latest_image.shape[0]).astype(int)

        valid_indices = (u >= 0) & (u < self.latest_image.shape[1]) & (v >= 0) & (v < self.latest_image.shape[0])
        u = u[valid_indices].get()
        v = v[valid_indices].get()
        x = x[valid_indices]
        y = y[valid_indices]

        updated_cells = set()

        # Define the range of green values in BGR format
        lower_green = np.array([35, 60, 70])
        upper_green = np.array([85, 255, 255])

        for i in range(len(u)):
            # Check if the pixel is within the green range
            if np.all(lower_green <= self.latest_image[v[i], u[i]]) and np.all(self.latest_image[v[i], u[i]] <= upper_green):
                map_x = int((x[i].get() - self.origin_x) / self.resolution)
                map_y = int((y[i].get() - self.origin_y) / self.resolution)
                if 0 <= map_x < self.width and 0 <= map_y < self.height:
                    self.costmap[map_y, map_x] = 100  # Mark the cell as occupied
                    updated_cells.add((map_x, map_y))

        # Clear cells that are no longer green
        cells_to_clear = self.previous_updated_cells - updated_cells
        for cell in cells_to_clear:
            self.costmap[cell[1], cell[0]] = 0  # Clear the cell

        self.previous_updated_cells = updated_cells

        if updated_cells or cells_to_clear:
            self.publish_costmap_update(updated_cells | cells_to_clear)

    def publish_costmap(self):
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

        occupancy_grid.data = cp.asnumpy(self.costmap).flatten().tolist()
        self.publisher_.publish(occupancy_grid)

    def publish_costmap_update(self, updated_cells):
        if not updated_cells:
            return

        min_x = min(cell[0] for cell in updated_cells)
        min_y = min(cell[1] for cell in updated_cells)
        max_x = max(cell[0] for cell in updated_cells)
        max_y = max(cell[1] for cell in updated_cells)

        width = max_x - min_x + 1
        height = max_y - min_y + 1

        data = []
        for y in range(min_y, min_y + height):
            for x in range(min_x, min_x + width):
                data.append(int(self.costmap[y, x].get()))

        occupancy_grid_update = OccupancyGridUpdate()
        occupancy_grid_update.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid_update.header.frame_id = "map"
        occupancy_grid_update.x = min_x
        occupancy_grid_update.y = min_y
        occupancy_grid_update.width = width
        occupancy_grid_update.height = height
        occupancy_grid_update.data = data

        self.update_publisher_.publish(occupancy_grid_update)

def main(args=None):
    rclpy.init(args=args)
    costmap_node = CostmapNode()
    rclpy.spin(costmap_node)
    costmap_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()