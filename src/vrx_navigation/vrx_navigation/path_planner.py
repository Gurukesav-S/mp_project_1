import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
from std_msgs.msg import Int32
from vrx_navigation.search_algorithms import DStarLitePlanner 
from scipy.spatial.transform import Rotation as R
import time

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        
        self.goal_pos = (155, 135) 
        self.current_pos = None
        self.current_yaw = 0.0
        self.grid = None
        self.cell_size = 10.0
        self.path = []
        self.current_wp_idx = 0
        self.wp_rad = 4.0 
        
        self.planner = None
        self.map_changed = False

        self.create_subscription(OccupancyGrid, '/map_grid', self.map_callback, 10)
        self.create_subscription(Odometry, '/kf/odom', self.odom_callback, 10)
        
        self.goal_pub = self.create_publisher(PoseStamped, '/pid/goal', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.expanded_pub = self.create_publisher(Int32, '/expanded_nodes', 10) 
        
        self.create_timer(0.2, self.guidance_loop)

    def map_callback(self, msg):
        rows = msg.info.height
        cols = msg.info.width
        self.cell_size = msg.info.resolution
        new_grid = (np.array(msg.data).reshape((rows, cols)) > 50).astype(int) 

        if self.grid is None:
            self.grid = new_grid
            self.map_changed = True
        elif not np.array_equal(self.grid, new_grid):
            self.grid = new_grid
            self.map_changed = True

    def odom_callback(self, msg):
        self.current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.current_yaw = R.from_quat(q).as_euler('xyz')[2]

    def world_to_grid(self, x, y):
        return int(y // self.cell_size), int(x // self.cell_size)

    def grid_to_world(self, row, col):
        return col * self.cell_size + (self.cell_size/2), row * self.cell_size + (self.cell_size/2)

    def hold_position(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = self.current_pos[0]
        msg.pose.position.y = self.current_pos[1]
        
        q = R.from_euler('z', self.current_yaw).as_quat()
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        
        self.goal_pub.publish(msg)

    def publish_planned_path(self, grid_path, expansions):
        exp_msg = Int32()
        exp_msg.data = expansions
        self.expanded_pub.publish(exp_msg)
        
        if not grid_path:
            self.get_logger().warn("No path found! ASV blocked. Standing by...")
            self.path = []
            self.hold_position()
            
            empty_path = Path()
            empty_path.header.stamp = self.get_clock().now().to_msg()
            empty_path.header.frame_id = 'odom'
            self.path_pub.publish(empty_path)
            return

        self.path = [self.grid_to_world(r, c) for r, c in grid_path]
        self.current_wp_idx = 1 if len(self.path) > 1 else 0
        
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        for wx, wy in self.path:
            pose = PoseStamped()
            pose.pose.position.x = float(wx)
            pose.pose.position.y = float(wy)
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def calculate_los(self, prev_wp, current_wp, current_pos):
        x1, y1 = prev_wp[0], prev_wp[1]
        x2, y2 = current_wp[0], current_wp[1]
        x3, y3 = current_pos[0], current_pos[1]
        
        dist = np.linalg.norm([x3 - x2, y3 - y2])
        denom = (x2 - x1) ** 2 + (y2 - y1) ** 2
        
        if denom < 1e-6:
            x5, y5 = x2, y2
            yaw_desired = np.arctan2(y2 - y3, x2 - x3)
        else:
            l = ((x2 - x1) * (x3 - x1) + (y2 - y1) * (y3 - y1)) / denom
            x4, y4 = l * (x2 - x1) + x1, l * (y2 - y1) + y1
            d2 = np.linalg.norm([x4 - x2, y4 - y2])
            delta = max(min(5.0, dist), 3.0)
            
            if d2 < 1e-6:
                x5, y5 = x2, y2
            else:
                x5, y5 = ((d2 - delta) * x4 + delta * x2) / d2, ((d2 - delta) * y4 + delta * y2) / d2
            
            yaw_desired = np.arctan2(y2 - y1, x2 - x1)
            
        if dist < 5.0:
            x5, y5 = x2, y2

        msg_pid = PoseStamped()
        msg_pid.header.stamp = self.get_clock().now().to_msg()
        msg_pid.header.frame_id = 'odom'
        msg_pid.pose.position.x = float(x5)
        msg_pid.pose.position.y = float(y5)
        msg_pid.pose.position.z = 0.0
        
        quat = R.from_euler('xyz', [0, 0, yaw_desired]).as_quat()
        msg_pid.pose.orientation.x = quat[0]
        msg_pid.pose.orientation.y = quat[1]
        msg_pid.pose.orientation.z = quat[2]
        msg_pid.pose.orientation.w = quat[3]
        
        return msg_pid

    def guidance_loop(self):
        if self.grid is None or self.current_pos is None:
            return

        r, c = self.world_to_grid(*self.current_pos)
        r = max(0, min(r, self.grid.shape[0]-1))
        c = max(0, min(c, self.grid.shape[1]-1))
        start_grid = (r, c)
        
        # --- THE FIX: SNAP TO FREE SPACE ---
        # If the ASV drifts into an obstacle boundary, snap the start_grid to the nearest safe water cell
        if self.grid[start_grid[0], start_grid[1]] == 1:
            best_dist = float('inf')
            best_node = start_grid
            for r_idx in range(self.grid.shape[0]):
                for c_idx in range(self.grid.shape[1]):
                    if self.grid[r_idx, c_idx] == 0:
                        dist = math.hypot(r_idx - start_grid[0], c_idx - start_grid[1])
                        if dist < best_dist:
                            best_dist = dist
                            best_node = (r_idx, c_idx)
            start_grid = best_node
        # ------------------------------------

        goal_grid = self.world_to_grid(*self.goal_pos)

        if self.planner is None:
            self.planner = DStarLitePlanner(self.grid, start_grid, goal_grid)
            grid_path, exp = self.planner.extract_path(start_grid)
            self.publish_planned_path(grid_path, exp)
            return

        if self.map_changed:
            self.map_changed = False
            self.planner.update_map(self.grid, start_grid)
            grid_path, exp = self.planner.extract_path(start_grid)
            self.publish_planned_path(grid_path, exp)

        if self.path and self.current_wp_idx < len(self.path):
            current_wp = self.path[self.current_wp_idx]
            
            if self.current_wp_idx == 0:
                prev_wp = self.current_pos
            else:
                prev_wp = self.path[self.current_wp_idx - 1]
            
            target_x, target_y = current_wp
            dist = np.hypot(target_x - self.current_pos[0], target_y - self.current_pos[1])
            
            if dist < self.wp_rad:
                self.current_wp_idx += 1
                return

            msg = self.calculate_los(prev_wp, current_wp, self.current_pos)
            self.goal_pub.publish(msg)

def main():
    rclpy.init()
    time.sleep(3)
    node = PathPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import OccupancyGrid, Odometry, Path
# from geometry_msgs.msg import PoseStamped
# import numpy as np
# from std_msgs.msg import Int32
# from vrx_navigation.search_algorithms import DStarLitePlanner 
# from scipy.spatial.transform import Rotation as R
# import time

# class PathPlannerNode(Node):
#     def __init__(self):
#         super().__init__('path_planner_node')
        
#         self.goal_pos = (155, 135) 
#         self.current_pos = None
#         self.current_yaw = 0.0
#         self.grid = None
#         self.cell_size = 10.0
#         self.path = []
#         self.current_wp_idx = 0
#         self.wp_rad = 4.0 
        
#         self.planner = None
#         self.map_changed = False

#         self.create_subscription(OccupancyGrid, '/map_grid', self.map_callback, 10)
#         self.create_subscription(Odometry, '/kf/odom', self.odom_callback, 10)
        
#         self.goal_pub = self.create_publisher(PoseStamped, '/pid/goal', 10)
#         self.path_pub = self.create_publisher(Path, '/planned_path', 10)
#         self.expanded_pub = self.create_publisher(Int32, '/expanded_nodes', 10) 
        
#         self.create_timer(0.2, self.guidance_loop)

#     def map_callback(self, msg):
#         rows = msg.info.height
#         cols = msg.info.width
#         self.cell_size = msg.info.resolution
#         new_grid = (np.array(msg.data).reshape((rows, cols)) > 50).astype(int) 

#         # THE FIX: Safely detects changes without getting overwritten
#         if self.grid is None:
#             self.grid = new_grid
#             self.map_changed = True
#         elif not np.array_equal(self.grid, new_grid):
#             self.grid = new_grid
#             self.map_changed = True

#     def odom_callback(self, msg):
#         self.current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
#         q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
#              msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
#         self.current_yaw = R.from_quat(q).as_euler('xyz')[2]

#     def world_to_grid(self, x, y):
#         return int(y // self.cell_size), int(x // self.cell_size)

#     def grid_to_world(self, row, col):
#         return col * self.cell_size + (self.cell_size/2), row * self.cell_size + (self.cell_size/2)

#     def hold_position(self):
#         msg = PoseStamped()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = 'odom'
#         msg.pose.position.x = self.current_pos[0]
#         msg.pose.position.y = self.current_pos[1]
        
#         q = R.from_euler('z', self.current_yaw).as_quat()
#         msg.pose.orientation.x = q[0]
#         msg.pose.orientation.y = q[1]
#         msg.pose.orientation.z = q[2]
#         msg.pose.orientation.w = q[3]
        
#         self.goal_pub.publish(msg)

#     def publish_planned_path(self, grid_path, expansions):
#         exp_msg = Int32()
#         exp_msg.data = expansions
#         self.expanded_pub.publish(exp_msg)
        
#         if not grid_path:
#             self.get_logger().warn("No path found! Walled off. Standing by...")
#             self.path = []
#             self.hold_position()
            
#             empty_path = Path()
#             empty_path.header.stamp = self.get_clock().now().to_msg()
#             empty_path.header.frame_id = 'odom'
#             self.path_pub.publish(empty_path)
#             return

#         self.path = [self.grid_to_world(r, c) for r, c in grid_path]
#         self.current_wp_idx = 1 if len(self.path) > 1 else 0
        
#         path_msg = Path()
#         path_msg.header.stamp = self.get_clock().now().to_msg()
#         path_msg.header.frame_id = 'odom'
#         for wx, wy in self.path:
#             pose = PoseStamped()
#             pose.pose.position.x = float(wx)
#             pose.pose.position.y = float(wy)
#             path_msg.poses.append(pose)
#         self.path_pub.publish(path_msg)

#     def calculate_los(self, prev_wp, current_wp, current_pos):
#         x1, y1 = prev_wp[0], prev_wp[1]
#         x2, y2 = current_wp[0], current_wp[1]
#         x3, y3 = current_pos[0], current_pos[1]
        
#         dist = np.linalg.norm([x3 - x2, y3 - y2])
#         denom = (x2 - x1) ** 2 + (y2 - y1) ** 2
        
#         if denom < 1e-6:
#             x5, y5 = x2, y2
#             yaw_desired = np.arctan2(y2 - y3, x2 - x3)
#         else:
#             l = ((x2 - x1) * (x3 - x1) + (y2 - y1) * (y3 - y1)) / denom
#             x4, y4 = l * (x2 - x1) + x1, l * (y2 - y1) + y1
#             d2 = np.linalg.norm([x4 - x2, y4 - y2])
#             delta = max(min(5.0, dist), 3.0)
            
#             if d2 < 1e-6:
#                 x5, y5 = x2, y2
#             else:
#                 x5, y5 = ((d2 - delta) * x4 + delta * x2) / d2, ((d2 - delta) * y4 + delta * y2) / d2
            
#             yaw_desired = np.arctan2(y2 - y1, x2 - x1)
            
#         if dist < 5.0:
#             x5, y5 = x2, y2

#         msg_pid = PoseStamped()
#         msg_pid.header.stamp = self.get_clock().now().to_msg()
#         msg_pid.header.frame_id = 'odom'
#         msg_pid.pose.position.x = float(x5)
#         msg_pid.pose.position.y = float(y5)
#         msg_pid.pose.position.z = 0.0
        
#         quat = R.from_euler('xyz', [0, 0, yaw_desired]).as_quat()
#         msg_pid.pose.orientation.x = quat[0]
#         msg_pid.pose.orientation.y = quat[1]
#         msg_pid.pose.orientation.z = quat[2]
#         msg_pid.pose.orientation.w = quat[3]
        
#         return msg_pid

#     def guidance_loop(self):
#         if self.grid is None or self.current_pos is None:
#             return

#         r, c = self.world_to_grid(*self.current_pos)
#         r = max(0, min(r, self.grid.shape[0]-1))
#         c = max(0, min(c, self.grid.shape[1]-1))
#         start_grid = (r, c)
#         goal_grid = self.world_to_grid(*self.goal_pos)

#         if self.planner is None:
#             self.planner = DStarLitePlanner(self.grid, start_grid, goal_grid)
#             grid_path, exp = self.planner.extract_path(start_grid)
#             self.publish_planned_path(grid_path, exp)
#             return

#         # THE FIX: Pushing the updated grid to the planner cleanly
#         if self.map_changed:
#             self.map_changed = False
#             self.planner.update_map(self.grid, start_grid)
#             grid_path, exp = self.planner.extract_path(start_grid)
#             self.publish_planned_path(grid_path, exp)

#         if self.path and self.current_wp_idx < len(self.path):
#             current_wp = self.path[self.current_wp_idx]
            
#             if self.current_wp_idx == 0:
#                 prev_wp = self.current_pos
#             else:
#                 prev_wp = self.path[self.current_wp_idx - 1]
            
#             target_x, target_y = current_wp
#             dist = np.hypot(target_x - self.current_pos[0], target_y - self.current_pos[1])
            
#             if dist < self.wp_rad:
#                 self.current_wp_idx += 1
#                 return

#             msg = self.calculate_los(prev_wp, current_wp, self.current_pos)
#             self.goal_pub.publish(msg)

# def main():
#     rclpy.init()
#     time.sleep(3)
#     node = PathPlannerNode()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()





# # import rclpy
# # from rclpy.node import Node
# # from nav_msgs.msg import OccupancyGrid, Odometry, Path
# # from geometry_msgs.msg import PoseStamped
# # import numpy as np
# # from std_msgs.msg import Int32
# # from vrx_navigation.search_algorithms import DStarLitePlanner 
# # from scipy.spatial.transform import Rotation as R
# # import time

# # class PathPlannerNode(Node):
# #     def __init__(self):
# #         super().__init__('path_planner_node')
        
# #         self.goal_pos = (155, 135) 
# #         self.current_pos = None
# #         self.current_yaw = 0.0
# #         self.grid = None
# #         self.cell_size = 10.0
# #         self.path = []
# #         self.current_wp_idx = 0
# #         self.wp_rad = 4.0 
        
# #         self.planner = None
# #         self.changed_cells = []
# #         self.map_changed = False

# #         self.create_subscription(OccupancyGrid, '/map_grid', self.map_callback, 10)
# #         self.create_subscription(Odometry, '/kf/odom', self.odom_callback, 10)
        
# #         self.goal_pub = self.create_publisher(PoseStamped, '/pid/goal', 10)
# #         self.path_pub = self.create_publisher(Path, '/planned_path', 10)
# #         self.expanded_pub = self.create_publisher(Int32, '/expanded_nodes', 10) 
        
# #         self.create_timer(0.2, self.guidance_loop)

# #     def map_callback(self, msg):
# #         rows = msg.info.height
# #         cols = msg.info.width
# #         self.cell_size = msg.info.resolution
# #         new_grid = (np.array(msg.data).reshape((rows, cols)) > 50).astype(int) 

# #         if self.grid is None:
# #             self.grid = new_grid
# #             self.map_changed = True
# #         elif not np.array_equal(self.grid, new_grid):
# #             changed_indices = np.argwhere(self.grid != new_grid)
# #             self.changed_cells = [(r, c) for r, c in changed_indices]
# #             self.grid = new_grid
# #             self.map_changed = True

# #     def odom_callback(self, msg):
# #         self.current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
# #         q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
# #              msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
# #         self.current_yaw = R.from_quat(q).as_euler('xyz')[2]

# #     def world_to_grid(self, x, y):
# #         return int(y // self.cell_size), int(x // self.cell_size)

# #     def grid_to_world(self, row, col):
# #         return col * self.cell_size + (self.cell_size/2), row * self.cell_size + (self.cell_size/2)

# #     def hold_position(self):
# #         msg = PoseStamped()
# #         msg.header.stamp = self.get_clock().now().to_msg()
# #         msg.header.frame_id = 'odom'
# #         msg.pose.position.x = self.current_pos[0]
# #         msg.pose.position.y = self.current_pos[1]
        
# #         q = R.from_euler('z', self.current_yaw).as_quat()
# #         msg.pose.orientation.x = q[0]
# #         msg.pose.orientation.y = q[1]
# #         msg.pose.orientation.z = q[2]
# #         msg.pose.orientation.w = q[3]
        
# #         self.goal_pub.publish(msg)

# #     def publish_planned_path(self, grid_path, expansions):
# #         exp_msg = Int32()
# #         exp_msg.data = expansions
# #         self.expanded_pub.publish(exp_msg)
        
# #         if not grid_path:
# #             self.get_logger().warn("No path found! Walled off. Standing by...")
# #             self.path = []
# #             self.hold_position()
            
# #             empty_path = Path()
# #             empty_path.header.stamp = self.get_clock().now().to_msg()
# #             empty_path.header.frame_id = 'odom'
# #             self.path_pub.publish(empty_path)
# #             return

# #         self.path = [self.grid_to_world(r, c) for r, c in grid_path]
# #         self.current_wp_idx = 1 if len(self.path) > 1 else 0
        
# #         path_msg = Path()
# #         path_msg.header.stamp = self.get_clock().now().to_msg()
# #         path_msg.header.frame_id = 'odom'
# #         for wx, wy in self.path:
# #             pose = PoseStamped()
# #             pose.pose.position.x = float(wx)
# #             pose.pose.position.y = float(wy)
# #             path_msg.poses.append(pose)
# #         self.path_pub.publish(path_msg)

# #     def calculate_los(self, prev_wp, current_wp, current_pos):
# #         x1, y1 = prev_wp[0], prev_wp[1]
# #         x2, y2 = current_wp[0], current_wp[1]
# #         x3, y3 = current_pos[0], current_pos[1]
        
# #         dist = np.linalg.norm([x3 - x2, y3 - y2])
# #         denom = (x2 - x1) ** 2 + (y2 - y1) ** 2
        
# #         if denom < 1e-6:
# #             x5, y5 = x2, y2
# #             yaw_desired = np.arctan2(y2 - y3, x2 - x3)
# #         else:
# #             l = ((x2 - x1) * (x3 - x1) + (y2 - y1) * (y3 - y1)) / denom
# #             x4, y4 = l * (x2 - x1) + x1, l * (y2 - y1) + y1
# #             d2 = np.linalg.norm([x4 - x2, y4 - y2])
# #             delta = max(min(5.0, dist), 3.0)
            
# #             if d2 < 1e-6:
# #                 x5, y5 = x2, y2
# #             else:
# #                 x5, y5 = ((d2 - delta) * x4 + delta * x2) / d2, ((d2 - delta) * y4 + delta * y2) / d2
            
# #             yaw_desired = np.arctan2(y2 - y1, x2 - x1)
            
# #         if dist < 5.0:
# #             x5, y5 = x2, y2

# #         msg_pid = PoseStamped()
# #         msg_pid.header.stamp = self.get_clock().now().to_msg()
# #         msg_pid.header.frame_id = 'odom'
# #         msg_pid.pose.position.x = float(x5)
# #         msg_pid.pose.position.y = float(y5)
# #         msg_pid.pose.position.z = 0.0
        
# #         quat = R.from_euler('xyz', [0, 0, yaw_desired]).as_quat()
# #         msg_pid.pose.orientation.x = quat[0]
# #         msg_pid.pose.orientation.y = quat[1]
# #         msg_pid.pose.orientation.z = quat[2]
# #         msg_pid.pose.orientation.w = quat[3]
        
# #         return msg_pid

# #     def guidance_loop(self):
# #         if self.grid is None or self.current_pos is None:
# #             return

# #         r, c = self.world_to_grid(*self.current_pos)
# #         r = max(0, min(r, self.grid.shape[0]-1))
# #         c = max(0, min(c, self.grid.shape[1]-1))
# #         start_grid = (r, c)
# #         goal_grid = self.world_to_grid(*self.goal_pos)

# #         if self.planner is None:
# #             self.planner = DStarLitePlanner(self.grid, start_grid, goal_grid)
# #             grid_path, exp = self.planner.extract_path(start_grid)
# #             self.publish_planned_path(grid_path, exp)
# #             return

# #         if self.map_changed:
# #             self.map_changed = False
# #             self.planner.update_map(self.grid, start_grid, self.changed_cells)
# #             self.changed_cells = []
# #             grid_path, exp = self.planner.extract_path(start_grid)
# #             self.publish_planned_path(grid_path, exp)

# #         if self.path and self.current_wp_idx < len(self.path):
# #             current_wp = self.path[self.current_wp_idx]
            
# #             if self.current_wp_idx == 0:
# #                 prev_wp = self.current_pos
# #             else:
# #                 prev_wp = self.path[self.current_wp_idx - 1]
            
# #             target_x, target_y = current_wp
# #             dist = np.hypot(target_x - self.current_pos[0], target_y - self.current_pos[1])
            
# #             if dist < self.wp_rad:
# #                 self.current_wp_idx += 1
# #                 return

# #             msg = self.calculate_los(prev_wp, current_wp, self.current_pos)
# #             self.goal_pub.publish(msg)

# # def main():
# #     rclpy.init()
# #     time.sleep(3)
# #     node = PathPlannerNode()
# #     rclpy.spin(node)
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()





# # import rclpy
# # from rclpy.node import Node
# # from nav_msgs.msg import OccupancyGrid, Odometry, Path
# # from geometry_msgs.msg import PoseStamped
# # import numpy as np
# # from std_msgs.msg import Int32
# # from vrx_navigation.search_algorithms import DStarLitePlanner 
# # from scipy.spatial.transform import Rotation as R
# # import time

# # class PathPlannerNode(Node):
# #     def __init__(self):
# #         super().__init__('path_planner_node')
        
# #         self.goal_pos = (155, 135) 
# #         self.current_pos = None
# #         self.current_yaw = 0.0
# #         self.grid = None
# #         self.cell_size = 10.0
# #         self.path = []
# #         self.current_wp_idx = 0
        
# #         # --- D* LITE INTEGRATION ---
# #         self.planner = None
# #         self.changed_cells = []
# #         self.map_changed = False

# #         self.create_subscription(OccupancyGrid, '/map_grid', self.map_callback, 10)
# #         self.create_subscription(Odometry, '/kf/odom', self.odom_callback, 10)
        
# #         self.goal_pub = self.create_publisher(PoseStamped, '/pid/goal', 10)
# #         self.path_pub = self.create_publisher(Path, '/planned_path', 10)
# #         self.expanded_pub = self.create_publisher(Int32, '/expanded_nodes', 10) 
        
# #         self.create_timer(0.2, self.guidance_loop)

# #     def map_callback(self, msg):
# #         rows = msg.info.height
# #         cols = msg.info.width
# #         self.cell_size = msg.info.resolution
# #         new_grid = (np.array(msg.data).reshape((rows, cols)) > 50).astype(int) 

# #         # Initial map load
# #         if self.grid is None:
# #             self.grid = new_grid
# #             self.map_changed = True
# #         # Detect diff for D* Lite repair
# #         elif not np.array_equal(self.grid, new_grid):
# #             changed_indices = np.argwhere(self.grid != new_grid)
# #             self.changed_cells = [(r, c) for r, c in changed_indices]
# #             self.grid = new_grid
# #             self.map_changed = True

# #     def odom_callback(self, msg):
# #         self.current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
# #         q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
# #              msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
# #         self.current_yaw = R.from_quat(q).as_euler('xyz')[2]

# #     def world_to_grid(self, x, y):
# #         return int(y // self.cell_size), int(x // self.cell_size)

# #     def grid_to_world(self, row, col):
# #         return col * self.cell_size + (self.cell_size/2), row * self.cell_size + (self.cell_size/2)

# #     def hold_position(self):
# #         msg = PoseStamped()
# #         msg.header.stamp = self.get_clock().now().to_msg()
# #         msg.header.frame_id = 'odom'
# #         msg.pose.position.x = self.current_pos[0]
# #         msg.pose.position.y = self.current_pos[1]
        
# #         q = R.from_euler('z', self.current_yaw).as_quat()
# #         msg.pose.orientation.x = q[0]
# #         msg.pose.orientation.y = q[1]
# #         msg.pose.orientation.z = q[2]
# #         msg.pose.orientation.w = q[3]
        
# #         self.goal_pub.publish(msg)

# #     def publish_planned_path(self, grid_path, expansions):
# #         exp_msg = Int32()
# #         exp_msg.data = expansions
# #         self.expanded_pub.publish(exp_msg)
        
# #         if not grid_path:
# #             self.get_logger().warn("No path found! Walled off. Standing by...")
# #             self.path = []
# #             self.hold_position()
            
# #             empty_path = Path()
# #             empty_path.header.stamp = self.get_clock().now().to_msg()
# #             empty_path.header.frame_id = 'odom'
# #             self.path_pub.publish(empty_path)
# #             return

# #         self.path = [self.grid_to_world(r, c) for r, c in grid_path]
# #         self.current_wp_idx = 0
        
# #         path_msg = Path()
# #         path_msg.header.stamp = self.get_clock().now().to_msg()
# #         path_msg.header.frame_id = 'odom'
# #         for wx, wy in self.path:
# #             pose = PoseStamped()
# #             pose.pose.position.x = float(wx)
# #             pose.pose.position.y = float(wy)
# #             path_msg.poses.append(pose)
# #         self.path_pub.publish(path_msg)

# #     def guidance_loop(self):
# #         if self.grid is None or self.current_pos is None:
# #             return

# #         r, c = self.world_to_grid(*self.current_pos)
# #         r = max(0, min(r, self.grid.shape[0]-1))
# #         c = max(0, min(c, self.grid.shape[1]-1))
# #         start_grid = (r, c)
# #         goal_grid = self.world_to_grid(*self.goal_pos)

# #         # 1. INITIALIZE D* LITE ONCE
# #         if self.planner is None:
# #             self.get_logger().info("Initializing Persistent D* Lite Planner...")
# #             self.planner = DStarLitePlanner(self.grid, start_grid, goal_grid)
# #             grid_path, exp = self.planner.extract_path(start_grid)
# #             self.publish_planned_path(grid_path, exp)
# #             return

# #         # 2. IF MAP CHANGED, REPAIR TREE
# #         if self.map_changed:
# #             self.map_changed = False
# #             self.get_logger().info(f"Map shift detected! Repairing D* Lite tree using {len(self.changed_cells)} changed cells...")
            
# #             self.planner.update_map(self.grid, start_grid, self.changed_cells)
# #             self.changed_cells = []
            
# #             grid_path, exp = self.planner.extract_path(start_grid)
# #             self.publish_planned_path(grid_path, exp)

# #         # 3. LOS / Waypoint following logic
# #         if self.path and self.current_wp_idx < len(self.path):
# #             target_x, target_y = self.path[self.current_wp_idx]
            
# #             dist = np.hypot(target_x - self.current_pos[0], target_y - self.current_pos[1])
# #             if dist < 5.0:
# #                 self.current_wp_idx += 1
# #                 return

# #             msg = PoseStamped()
# #             msg.header.stamp = self.get_clock().now().to_msg()
# #             msg.header.frame_id = 'odom'
# #             msg.pose.position.x = target_x
# #             msg.pose.position.y = target_y
            
# #             bearing = np.arctan2(target_y - self.current_pos[1], target_x - self.current_pos[0])
# #             q = R.from_euler('z', bearing).as_quat()
# #             msg.pose.orientation.x = q[0]
# #             msg.pose.orientation.y = q[1]
# #             msg.pose.orientation.z = q[2]
# #             msg.pose.orientation.w = q[3]
            
# #             self.goal_pub.publish(msg)

# # def main():
# #     rclpy.init()
# #     time.sleep(3)
# #     node = PathPlannerNode()
# #     rclpy.spin(node)
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()
