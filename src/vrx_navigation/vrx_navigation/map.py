import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

class VRXGridPlotter(Node):
    def __init__(self):
        super().__init__('vrx_grid_plotter')
        self.map_pub = self.create_publisher(OccupancyGrid, '/map_grid', 10)
        self.create_timer(1.0, self.publish_grid_msg) # Publish map every second
        
        # 1. Hardcoded Grid Map (10x10m cells)
        # 0 = Free, 1 = Obstacle
        self.grid_map_inv = np.array([
            [0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0],
            [0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1],
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1],
            [0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0]
        ])
        self.grid_map = self.grid_map_inv[::-1]
    
        self.cell_size = 10.0  # meters
        self.offset = self.cell_size / 2.0  # 5.0m (to reach center)

        # ASV State Tracking
        self.path_x, self.path_y = [], []
        self.current_pos = [0.0, 0.0, 0.0] # x, y, yaw

        self.create_subscription(Odometry, '/kf/odom', self.odom_callback, 10)
        
        # Visualization Setup
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 7))
        self.create_timer(0.2, self.update_plot)

    def publish_grid_msg(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        
        # Metadata
        rows, cols = self.grid_map.shape
        msg.info.resolution = self.cell_size
        msg.info.width = cols
        msg.info.height = rows
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        
        # Flatten map and convert to ROS convention (0-100, -1 for unknown)
        # Your grid: 1 = Obstacle, 0 = Free -> ROS: 100 = Obstacle, 0 = Free
        flat_grid = (self.grid_map.flatten() * 100).astype(np.int8).tolist()
        msg.data = flat_grid
        self.map_pub.publish(msg)

    def odom_callback(self, msg):
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        
        q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.current_pos[2] = R.from_quat(q).as_euler('xyz')[2]
        
        self.path_x.append(self.current_pos[0])
        self.path_y.append(self.current_pos[1])

    def update_plot(self):
        self.ax.clear()
        if self.current_pos != [0.0, 0.0, 0.0]:
        # Calculate boundaries for the plot based on matrix shape
        # Rows = Y axis, Cols = X axis
            rows, cols = self.grid_map.shape
            extent = [0, cols * self.cell_size, 0, rows * self.cell_size]
            
            # Display the grid
            # cmap='binary' makes 1s black (obstacles) and 0s white
            self.ax.imshow(self.grid_map, cmap='binary', origin='lower', 
                        extent=extent, alpha=0.3)

            # Draw Grid Lines at multiples of 10 to visualize cells
            self.ax.set_xticks(np.arange(0, extent[1] + 1, self.cell_size))
            self.ax.set_yticks(np.arange(0, extent[3] + 1, self.cell_size))
            self.ax.grid(which='both', color='gray', linestyle='-', linewidth=0.5)

            # Plot Trajectory
            self.ax.plot(self.path_x, self.path_y, 'b-', label='Path', linewidth=1.5)

            # Plot ASV as an arrow (Heading)
            self.ax.quiver(self.current_pos[0], self.current_pos[1], 
                        np.cos(self.current_pos[2]), np.sin(self.current_pos[2]), 
                        color='red', scale=20, label='WAM-V')

            self.ax.set_title(f"VRX Navigation (Cell Size: {self.cell_size}m)")
            self.ax.legend()
            plt.draw()
            plt.pause(0.01)

def main():
    rclpy.init()
    node = VRXGridPlotter()
    try:
        time.sleep(2)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()