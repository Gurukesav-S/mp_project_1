"""does not reinput hardcoded map"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Header, Int32 
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
import random

class VRXGridPlotter(Node):
    def __init__(self):
        super().__init__('vrx_grid_plotter')
        self.map_pub = self.create_publisher(OccupancyGrid, '/map_grid', 10)
        self.create_timer(1.0, self.publish_grid_msg) 
        
        # =======================================================
        # --- TOGGLE SWITCH: Set True for Dynamic, False for Static ---
        self.is_dynamic = True
        # =======================================================

        # 1. Hardcoded Grid Map (10x10m cells)
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
        self.goal_grid = (7, 10) 
        
        # Only start the dynamic obstacle randomizer if the toggle is True!
        if self.is_dynamic:
            self.create_timer(60.0, self.update_dynamic_obstacles) 

        self.cell_size = 10.0  
        self.offset = self.cell_size / 2.0  

        # ASV State Tracking
        self.path_x, self.path_y = [], []
        self.current_pos = None 
        self.planned_x, self.planned_y = [], []
        self.expanded_nodes = 0

        self.create_subscription(Odometry, '/kf/odom', self.odom_callback, 10)
        self.create_subscription(Path, '/planned_path', self.path_callback, 10) 
        self.create_subscription(Int32, '/expanded_nodes', self.expanded_callback, 10) 
        
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 7))
        self.create_timer(0.2, self.update_plot)

    def update_dynamic_obstacles(self):
        # We DO NOT reset the map to base_grid. It evolves from its current state.
        safe_cells = set()
        safe_cells.add(self.goal_grid)
        
        # Guard against None when creating safe zones
        if self.current_pos is not None:
            asv_row = int(self.current_pos[1] // self.cell_size)
            asv_col = int(self.current_pos[0] // self.cell_size)
            
            # Target the ASV + all 8 surrounding cells
            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    r_safe = asv_row + dr
                    c_safe = asv_col + dc
                    
                    # Boundary check to prevent IndexErrors at the map's edges
                    if 0 <= r_safe < self.grid_map.shape[0] and 0 <= c_safe < self.grid_map.shape[1]:
                        # Register as safe so the randomizer CANNOT pick these cells
                        safe_cells.add((r_safe, c_safe))
                    
        candidates_0 = []
        candidates_1 = []
        rows, cols = self.grid_map.shape
        
        # Find candidates using the CURRENT grid map state
        for r in range(rows):
            for c in range(cols):
                if (r, c) not in safe_cells:
                    if self.grid_map[r, c] == 0:
                        candidates_0.append((r, c))
                    elif self.grid_map[r, c] == 1:
                        candidates_1.append((r, c))
                        
        num_to_flip_0 = min(2, len(candidates_0))
        num_to_flip_1 = min(2, len(candidates_1))
        
        flipped_0 = random.sample(candidates_0, num_to_flip_0)
        flipped_1 = random.sample(candidates_1, num_to_flip_1)
        
        for r, c in flipped_0:
            self.grid_map[r, c] = 1 
            
        for r, c in flipped_1:
            self.grid_map[r, c] = 0 
            
        self.get_logger().info(f"Map updated: Added {num_to_flip_0} dynamic obstacles, Opened {num_to_flip_1} new doors.")

    def publish_grid_msg(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        
        rows, cols = self.grid_map.shape
        msg.info.resolution = self.cell_size
        msg.info.width = cols
        msg.info.height = rows
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        
        flat_grid = (self.grid_map.flatten() * 100).astype(np.int8).tolist()
        msg.data = flat_grid
        self.map_pub.publish(msg)

    def odom_callback(self, msg):
        if self.current_pos is None:
            self.current_pos = [0.0, 0.0, 0.0]

        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.current_pos[2] = R.from_quat(q).as_euler('xyz')[2]
        
        self.path_x.append(self.current_pos[0])
        self.path_y.append(self.current_pos[1])

    def path_callback(self, msg):
        self.planned_x = [pose.pose.position.x for pose in msg.poses]
        self.planned_y = [pose.pose.position.y for pose in msg.poses]

    def expanded_callback(self, msg):
        self.expanded_nodes = msg.data 

    def update_plot(self):
        self.ax.clear()
        if self.current_pos is not None:
            rows, cols = self.grid_map.shape
            extent = [0, cols * self.cell_size, 0, rows * self.cell_size]
            
            self.ax.imshow(self.grid_map, cmap='binary', origin='lower', extent=extent, alpha=0.3)
            self.ax.set_xticks(np.arange(0, extent[1] + 1, self.cell_size))
            self.ax.set_yticks(np.arange(0, extent[3] + 1, self.cell_size))
            self.ax.grid(which='both', color='gray', linestyle='-', linewidth=0.5)
            
            if self.planned_x and self.planned_y:
                self.ax.plot(self.planned_x, self.planned_y, 'g--', label='Planned A* Path', linewidth=2.5)
                self.ax.plot(self.planned_x[-1], self.planned_y[-1], 'g*', markersize=15, label='Goal')
                
            self.ax.plot(self.path_x, self.path_y, 'b-', label='Path', linewidth=1.5)
            self.ax.quiver(self.current_pos[0], self.current_pos[1], 
                        np.cos(self.current_pos[2]), np.sin(self.current_pos[2]), 
                        color='red', scale=20, label='WAM-V')
                        
            # Update title to show the active mode
            mode = "Dynamic" if self.is_dynamic else "Static"
            self.ax.set_title(f"VRX Navigation ({mode} Mode) | A* Nodes Expanded: {self.expanded_nodes}")
            
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

"""Reinput hardcoded map"""
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import OccupancyGrid, Odometry, Path
# from std_msgs.msg import Header, Int32 
# import matplotlib.pyplot as plt
# import numpy as np
# from scipy.spatial.transform import Rotation as R
# import time
# import random

# class VRXGridPlotter(Node):
#     def __init__(self):
#         super().__init__('vrx_grid_plotter')
#         self.map_pub = self.create_publisher(OccupancyGrid, '/map_grid', 10)
#         self.create_timer(1.0, self.publish_grid_msg) 
        
#         # 1. Hardcoded Grid Map (10x10m cells)
#         self.grid_map_inv = np.array([
#             [0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0],
#             [0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0],
#             [0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1],
#             [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
#             [1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1],
#             [0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0],
#             [0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0]
#         ])
        
#         # --- DYNAMIC OBSTACLE SETUP ---
#         self.grid_map = self.grid_map_inv[::-1]
#         self.base_grid = np.copy(self.grid_map) 
#         self.create_timer(60.0, self.update_dynamic_obstacles) # Restored!
#         self.goal_grid = (7, 10) 
#         # ------------------------------

#         self.cell_size = 10.0  
#         self.offset = self.cell_size / 2.0  

#         # ASV State Tracking
#         self.path_x, self.path_y = [], []
#         self.current_pos = None # FIXED: Must start as None
#         self.planned_x, self.planned_y = [], []
#         self.expanded_nodes = 0

#         self.create_subscription(Odometry, '/kf/odom', self.odom_callback, 10)
#         self.create_subscription(Path, '/planned_path', self.path_callback, 10) 
#         self.create_subscription(Int32, '/expanded_nodes', self.expanded_callback, 10) 
        
#         plt.ion()
#         self.fig, self.ax = plt.subplots(figsize=(10, 7))
#         self.create_timer(0.2, self.update_plot)

#     def update_dynamic_obstacles(self):
#         self.grid_map = np.copy(self.base_grid)
#         safe_cells = set()
#         safe_cells.add(self.goal_grid)
        
#         # Guard against None when creating safe zones
#         if self.current_pos is not None:
#             asv_row = int(self.current_pos[1] // self.cell_size)
#             asv_col = int(self.current_pos[0] // self.cell_size)
#             for dr in [-1, 0, 1]:
#                 for dc in [-1, 0, 1]:
#                     safe_cells.add((asv_row + dr, asv_col + dc))
                    
#         candidates_0 = []
#         candidates_1 = []
#         rows, cols = self.base_grid.shape
#         for r in range(rows):
#             for c in range(cols):
#                 if (r, c) not in safe_cells:
#                     if self.base_grid[r, c] == 0:
#                         candidates_0.append((r, c))
#                     elif self.base_grid[r, c] == 1:
#                         candidates_1.append((r, c))
                        
#         num_to_flip_0 = min(2, len(candidates_0))
#         num_to_flip_1 = min(2, len(candidates_1))
        
#         flipped_0 = random.sample(candidates_0, num_to_flip_0)
#         flipped_1 = random.sample(candidates_1, num_to_flip_1)
        
#         for r, c in flipped_0:
#             self.grid_map[r, c] = 1 
            
#         for r, c in flipped_1:
#             self.grid_map[r, c] = 0 
            
#         self.get_logger().info(f"Map updated: Added {num_to_flip_0} dynamic obstacles, Opened {num_to_flip_1} new doors.")

#     def publish_grid_msg(self):
#         msg = OccupancyGrid()
#         msg.header = Header()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = 'odom'
        
#         rows, cols = self.grid_map.shape
#         msg.info.resolution = self.cell_size
#         msg.info.width = cols
#         msg.info.height = rows
#         msg.info.origin.position.x = 0.0
#         msg.info.origin.position.y = 0.0
        
#         flat_grid = (self.grid_map.flatten() * 100).astype(np.int8).tolist()
#         msg.data = flat_grid
#         self.map_pub.publish(msg)

#     def odom_callback(self, msg):
#         if self.current_pos is None:
#             self.current_pos = [0.0, 0.0, 0.0]

#         self.current_pos[0] = msg.pose.pose.position.x
#         self.current_pos[1] = msg.pose.pose.position.y
#         q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
#              msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
#         self.current_pos[2] = R.from_quat(q).as_euler('xyz')[2]
        
#         self.path_x.append(self.current_pos[0])
#         self.path_y.append(self.current_pos[1])

#     def path_callback(self, msg):
#         self.planned_x = [pose.pose.position.x for pose in msg.poses]
#         self.planned_y = [pose.pose.position.y for pose in msg.poses]

#     def expanded_callback(self, msg):
#         self.expanded_nodes = msg.data 

#     def update_plot(self):
#         self.ax.clear()
#         if self.current_pos is not None:
#             rows, cols = self.grid_map.shape
#             extent = [0, cols * self.cell_size, 0, rows * self.cell_size]
            
#             self.ax.imshow(self.grid_map, cmap='binary', origin='lower', extent=extent, alpha=0.3)
#             self.ax.set_xticks(np.arange(0, extent[1] + 1, self.cell_size))
#             self.ax.set_yticks(np.arange(0, extent[3] + 1, self.cell_size))
#             self.ax.grid(which='both', color='gray', linestyle='-', linewidth=0.5)
            
#             if self.planned_x and self.planned_y:
#                 self.ax.plot(self.planned_x, self.planned_y, 'g--', label='Planned A* Path', linewidth=2.5)
#                 self.ax.plot(self.planned_x[-1], self.planned_y[-1], 'g*', markersize=15, label='Goal')
                
#             self.ax.plot(self.path_x, self.path_y, 'b-', label='Path', linewidth=1.5)
#             self.ax.quiver(self.current_pos[0], self.current_pos[1], 
#                         np.cos(self.current_pos[2]), np.sin(self.current_pos[2]), 
#                         color='red', scale=20, label='WAM-V')
                        
#             self.ax.set_title(f"VRX Navigation (Cell Size: {self.cell_size}m) | A* Nodes Expanded: {self.expanded_nodes}")
#             self.ax.legend()
#             plt.draw()
#             plt.pause(0.01)

# def main():
#     rclpy.init()
#     node = VRXGridPlotter()
#     try:
#         time.sleep(2)
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

