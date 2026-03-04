import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
from std_msgs.msg import Int32
from vrx_navigation.search_algorithms import a_star 
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
        
        self.map_changed = False
        self.total_planning_time = 0.0 
        self.total_expanded_nodes = 0 # NEW: Tracker for total expansions

        self.create_subscription(OccupancyGrid, '/map_grid', self.map_callback, 10)
        self.create_subscription(Odometry, '/kf/odom', self.odom_callback, 10)
        
        self.goal_pub = self.create_publisher(PoseStamped, '/pid/goal', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10) # Smoothed Path
        self.raw_path_pub = self.create_publisher(Path, '/raw_planned_path', 10) # NEW: Raw Path
        
        self.expanded_pub = self.create_publisher(Int32, '/expanded_nodes', 10) # Current expansions
        self.total_expanded_pub = self.create_publisher(Int32, '/total_expanded_nodes', 10) # NEW: Total expansions
        
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

    def is_line_of_sight_clear(self, r0, c0, r1, c1):
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        r = r0
        c = c0
        n = 1 + dr + dc
        r_inc = 1 if r1 > r0 else -1
        c_inc = 1 if c1 > c0 else -1
        error = dr - dc
        dr *= 2
        dc *= 2

        def is_wall(check_r, check_c):
            if 0 <= check_r < self.grid.shape[0] and 0 <= check_c < self.grid.shape[1]:
                return self.grid[check_r, check_c] == 1
            return False 

        for _ in range(n):
            if not (0 <= r < self.grid.shape[0] and 0 <= c < self.grid.shape[1]): return False
            if self.grid[r, c] == 1: return False
                
            if error > 0:
                if is_wall(r, c - c_inc) or is_wall(r + r_inc, c): return False
                r += r_inc
                error -= dc
            elif error < 0:
                if is_wall(r - r_inc, c) or is_wall(r, c + c_inc): return False
                c += c_inc
                error += dr
            else:
                if is_wall(r + r_inc, c) or is_wall(r, c + c_inc): return False
                r += r_inc
                c += c_inc
                error -= dc
                error += dr
                n -= 1
        return True

    def is_line_of_sight_clear_simple(self, r0, c0, r1, c1):
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        step_r = 1 if r0 < r1 else -1
        step_c = 1 if c0 < c1 else -1
        err = dr - dc

        while True:
            if not (0 <= r0 < self.grid.shape[0] and 0 <= c0 < self.grid.shape[1]): return False
            if self.grid[r0, c0] == 1: return False
            if r0 == r1 and c0 == c1: return True
                
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r0 += step_r
            if e2 < dr:
                err += dr
                c0 += step_c

    def publish_planned_path(self, grid_path, expansions, total_expansions):
        exp_msg = Int32()
        exp_msg.data = expansions
        self.expanded_pub.publish(exp_msg)
        
        tot_exp_msg = Int32()
        tot_exp_msg.data = total_expansions
        self.total_expanded_pub.publish(tot_exp_msg)
        
        if not grid_path:
            self.get_logger().warn("No path found! ASV blocked. Standing by...")
            self.path = []
            self.hold_position()
            
            empty_path = Path()
            empty_path.header.stamp = self.get_clock().now().to_msg()
            empty_path.header.frame_id = 'odom'
            self.path_pub.publish(empty_path)
            self.raw_path_pub.publish(empty_path)
            return

        # --- NEW: PUBLISH RAW GRID PATH ---
        raw_path_msg = Path()
        raw_path_msg.header.stamp = self.get_clock().now().to_msg()
        raw_path_msg.header.frame_id = 'odom'
        for r, c in grid_path:
            wx, wy = self.grid_to_world(r, c)
            pose = PoseStamped()
            pose.pose.position.x = float(wx)
            pose.pose.position.y = float(wy)
            raw_path_msg.poses.append(pose)
        self.raw_path_pub.publish(raw_path_msg)

        # --- BRESENHAM PATH SMOOTHING ---
        smoothed_path = [grid_path[0]]
        current_idx = 0
        
        while current_idx < len(grid_path) - 1:
            furthest_visible_idx = current_idx + 1
            for j in range(len(grid_path) - 1, current_idx, -1):
                r0, c0 = grid_path[current_idx]
                r1, c1 = grid_path[j]
                
                if self.is_line_of_sight_clear_simple(r0, c0, r1, c1):
                    furthest_visible_idx = j
                    break
            
            smoothed_path.append(grid_path[furthest_visible_idx])
            current_idx = furthest_visible_idx

        self.path = [self.grid_to_world(r, c) for r, c in smoothed_path]
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
            
        goal_grid = self.world_to_grid(*self.goal_pos)

        if self.path == [] or self.map_changed:
            self.map_changed = False
            
            t_start = time.perf_counter()
            grid_path, exp = a_star(self.grid, start_grid, goal_grid, None, 1)
            run_time = time.perf_counter() - t_start
            
            self.total_planning_time += run_time
            self.total_expanded_nodes += exp # Accumulate nodes
            
            self.get_logger().info(f"--- A* Search ---")
            self.get_logger().info(f"Run Time: {run_time:.5f} sec | Total Time: {self.total_planning_time:.5f} sec")
            
            self.publish_planned_path(grid_path, exp, self.total_expanded_nodes)
            return

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
#error
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import OccupancyGrid, Odometry, Path
# from geometry_msgs.msg import PoseStamped
# import numpy as np
# import math
# from std_msgs.msg import Int32
# from vrx_navigation.search_algorithms import a_star 
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
        
#         self.map_changed = False
#         self.total_planning_time = 0.0 # From D* Lite
#         self.total_expanded_nodes = 0 # NEW: Tracker for total expansions
#         self.create_subscription(OccupancyGrid, '/map_grid', self.map_callback, 10)
#         self.create_subscription(Odometry, '/kf/odom', self.odom_callback, 10)
        
#         self.goal_pub = self.create_publisher(PoseStamped, '/pid/goal', 10)
#         self.path_pub = self.create_publisher(Path, '/planned_path', 10)
#         self.raw_path_pub = self.create_publisher(Path, '/raw_planned_path', 10) # NEW: Raw Path
#         self.expanded_pub = self.create_publisher(Int32, '/expanded_nodes', 10) 
#         self.total_expanded_pub = self.create_publisher(Int32, '/total_expanded_nodes', 10) # NEW: Total expansions
#         self.create_timer(0.2, self.guidance_loop)

#     def map_callback(self, msg):
#         rows = msg.info.height
#         cols = msg.info.width
#         self.cell_size = msg.info.resolution
#         new_grid = (np.array(msg.data).reshape((rows, cols)) > 50).astype(int) 

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

#     # --- BRESENHAM FUNCTION 1 (Supercover - No corner cutting) ---
#     def is_line_of_sight_clear(self, r0, c0, r1, c1):
#         dr = abs(r1 - r0)
#         dc = abs(c1 - c0)
#         r = r0
#         c = c0
#         n = 1 + dr + dc
#         r_inc = 1 if r1 > r0 else -1
#         c_inc = 1 if c1 > c0 else -1
#         error = dr - dc
#         dr *= 2
#         dc *= 2

#         def is_wall(check_r, check_c):
#             if 0 <= check_r < self.grid.shape[0] and 0 <= check_c < self.grid.shape[1]:
#                 return self.grid[check_r, check_c] == 1
#             return False 

#         for _ in range(n):
#             if not (0 <= r < self.grid.shape[0] and 0 <= c < self.grid.shape[1]): return False
#             if self.grid[r, c] == 1: return False
                
#             if error > 0:
#                 if is_wall(r, c - c_inc) or is_wall(r + r_inc, c): return False
#                 r += r_inc
#                 error -= dc
#             elif error < 0:
#                 if is_wall(r - r_inc, c) or is_wall(r, c + c_inc): return False
#                 c += c_inc
#                 error += dr
#             else:
#                 if is_wall(r + r_inc, c) or is_wall(r, c + c_inc): return False
#                 r += r_inc
#                 c += c_inc
#                 error -= dc
#                 error += dr
#                 n -= 1
#         return True

#     # --- BRESENHAM FUNCTION 2 (Simple - Allows corner cutting) ---
#     def is_line_of_sight_clear_simple(self, r0, c0, r1, c1):
#         dr = abs(r1 - r0)
#         dc = abs(c1 - c0)
#         step_r = 1 if r0 < r1 else -1
#         step_c = 1 if c0 < c1 else -1
#         err = dr - dc

#         while True:
#             if not (0 <= r0 < self.grid.shape[0] and 0 <= c0 < self.grid.shape[1]): return False
#             if self.grid[r0, c0] == 1: return False
#             if r0 == r1 and c0 == c1: return True
                
#             e2 = 2 * err
#             if e2 > -dc:
#                 err -= dc
#                 r0 += step_r
#             if e2 < dr:
#                 err += dr
#                 c0 += step_c

#     def publish_planned_path(self, grid_path, expansions):
#         exp_msg = Int32()
#         exp_msg.data = expansions
#         self.expanded_pub.publish(exp_msg)
        
#         if not grid_path:
#             self.get_logger().warn("No path found! ASV blocked. Standing by...")
#             self.path = []
#             self.hold_position()
            
#             empty_path = Path()
#             empty_path.header.stamp = self.get_clock().now().to_msg()
#             empty_path.header.frame_id = 'odom'
#             self.path_pub.publish(empty_path)
#             return

#         # --- NEW: PUBLISH RAW GRID PATH ---
#         raw_path_msg = Path()
#         raw_path_msg.header.stamp = self.get_clock().now().to_msg()
#         raw_path_msg.header.frame_id = 'odom'
#         for r, c in grid_path:
#             wx, wy = self.grid_to_world(r, c)
#             pose = PoseStamped()
#             pose.pose.position.x = float(wx)
#             pose.pose.position.y = float(wy)
#             raw_path_msg.poses.append(pose)
#         self.raw_path_pub.publish(raw_path_msg)

#         # --- BRESENHAM PATH SMOOTHING ---
#         smoothed_path = [grid_path[0]]
#         current_idx = 0
        
#         while current_idx < len(grid_path) - 1:
#             furthest_visible_idx = current_idx + 1
#             for j in range(len(grid_path) - 1, current_idx, -1):
#                 r0, c0 = grid_path[current_idx]
#                 r1, c1 = grid_path[j]
                
#                 # Active: Using the Simple smoother. Swap to `is_line_of_sight_clear` for Supercover!
#                 if self.is_line_of_sight_clear_simple(r0, c0, r1, c1):
#                     furthest_visible_idx = j
#                     break
            
#             smoothed_path.append(grid_path[furthest_visible_idx])
#             current_idx = furthest_visible_idx

#         self.path = [self.grid_to_world(r, c) for r, c in smoothed_path]
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
        
#         # --- SNAP TO FREE SPACE (From D* Lite) ---
#         if self.grid[start_grid[0], start_grid[1]] == 1:
#             best_dist = float('inf')
#             best_node = start_grid
#             for r_idx in range(self.grid.shape[0]):
#                 for c_idx in range(self.grid.shape[1]):
#                     if self.grid[r_idx, c_idx] == 0:
#                         dist = math.hypot(r_idx - start_grid[0], c_idx - start_grid[1])
#                         if dist < best_dist:
#                             best_dist = dist
#                             best_node = (r_idx, c_idx)
#             start_grid = best_node
            
#         goal_grid = self.world_to_grid(*self.goal_pos)

#         # --- A* PLANNING WITH TIMERS ---
#         if self.path == [] or self.map_changed:
#             self.map_changed = False
            
#             t_start = time.perf_counter()
#             grid_path, exp = a_star(self.grid, start_grid, goal_grid, None, 1) # Heuristic = 1 (Euclidean)
#             run_time = time.perf_counter() - t_start
#             self.total_planning_time += run_time
#             self.total_expanded_nodes += exp

#             self.get_logger().info(f"--- A* Search ---")
#             self.get_logger().info(f"Run Time: {run_time:.5f} sec | Total Time: {self.total_planning_time:.5f} sec")
            
#             self.publish_planned_path(grid_path, exp, self.total_expanded_nodes)
#             return

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



# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import OccupancyGrid, Odometry, Path
# from geometry_msgs.msg import PoseStamped
# import numpy as np
# from std_msgs.msg import Int32
# from vrx_navigation.search_algorithms import a_star 
# from scipy.spatial.transform import Rotation as R
# import time

# class PathPlannerNode(Node):
#     def __init__(self):
#         super().__init__('path_planner_node')
        
#         self.goal_pos = (105, 75) # (x, y)
#         self.current_pos = None
#         self.current_yaw = 0.0
#         self.grid = None
#         self.cell_size = 10.0
#         self.path = []
#         self.current_wp_idx = 0
        
#         self.total_expanded_nodes = 0

#         # --- NEW: State Tracking Flags ---
#         self.map_changed = False
#         self.needs_replanning = True # Start true so it plans the first route

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

#         # --- NEW: Only react if the map actually changed structurally ---
#         if self.grid is None or not np.array_equal(self.grid, new_grid):
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

#     def guidance_loop(self):
#         if self.grid is None or self.current_pos is None:
#             return

#         # 1. Map Update Check
#         if self.map_changed:
#             self.map_changed = False # Consume flag
            
#             # If we have a path, verify it's not blocked
#             if self.path and self.current_wp_idx < len(self.path):
#                 path_blocked = False
#                 for i in range(self.current_wp_idx, len(self.path)):
#                     wx, wy = self.path[i]
#                     r, c = self.world_to_grid(wx, wy)
#                     if 0 <= r < self.grid.shape[0] and 0 <= c < self.grid.shape[1]:
#                         if self.grid[r, c] == 1:
#                             path_blocked = True
#                             break
                
#                 if path_blocked:
#                     self.get_logger().info("Path blocked! Clearing path.")
#                     self.path = [] 
#                     self.current_wp_idx = 0
                    
#                     empty_path = Path()
#                     empty_path.header.stamp = self.get_clock().now().to_msg()
#                     empty_path.header.frame_id = 'odom'
#                     self.path_pub.publish(empty_path)
            
#             # If we have NO path (either blocked or A* previously failed), request a replan
#             if not self.path:
#                 self.needs_replanning = True

#         # 2. Plan path ONLY if the replanning flag is True
#         if self.needs_replanning:
#             self.needs_replanning = False # Consume flag so we ONLY try once!
            
#             r, c = self.world_to_grid(*self.current_pos)
#             r = max(0, min(r, self.grid.shape[0]-1))
#             c = max(0, min(c, self.grid.shape[1]-1))
#             start_grid = (r, c)
#             goal_grid = self.world_to_grid(*self.goal_pos)
            
#             if self.grid[start_grid[0], start_grid[1]] == 1 or self.grid[goal_grid[0], goal_grid[1]] == 1:
#                 self.get_logger().warn("Start or Goal is blocked. Standing by...")
#                 self.hold_position()
#                 return

#             dummy_costs = np.ones_like(self.grid)
#             self.get_logger().info("Running A* Planner...")
#             grid_path, expanded_count = a_star(self.grid, start_grid, goal_grid, dummy_costs, 0)
            
#             self.total_expanded_nodes += expanded_count
#             exp_msg = Int32()
#             exp_msg.data = self.total_expanded_nodes
#             self.expanded_pub.publish(exp_msg)
            
#             if grid_path:
#                 self.path = [self.grid_to_world(r, c) for r, c in grid_path]
#                 self.get_logger().info(f"Path Planned: {len(self.path)} waypoints. Total Expanded: {self.total_expanded_nodes}")
                
#                 path_msg = Path()
#                 path_msg.header.stamp = self.get_clock().now().to_msg()
#                 path_msg.header.frame_id = 'odom'
#                 for wx, wy in self.path:
#                     pose = PoseStamped()
#                     pose.pose.position.x = float(wx)
#                     pose.pose.position.y = float(wy)
#                     path_msg.poses.append(pose)
#                 self.path_pub.publish(path_msg)
#             else:
#                 self.get_logger().warn("No path found! Waiting for map to change...")
#                 self.hold_position()
#                 return # Give up for now, wait for the map to change again!

#         # 3. LOS / Waypoint following logic
#         if self.path and self.current_wp_idx < len(self.path):
#             target_x, target_y = self.path[self.current_wp_idx]
            
#             dist = np.hypot(target_x - self.current_pos[0], target_y - self.current_pos[1])
#             if dist < 3.0:
#                 self.current_wp_idx += 1
#                 return

#             msg = PoseStamped()
#             msg.header.stamp = self.get_clock().now().to_msg()
#             msg.header.frame_id = 'odom'
#             msg.pose.position.x = target_x
#             msg.pose.position.y = target_y
            
#             bearing = np.arctan2(target_y - self.current_pos[1], target_x - self.current_pos[0])
#             q = R.from_euler('z', bearing).as_quat()
#             msg.pose.orientation.x = q[0]
#             msg.pose.orientation.y = q[1]
#             msg.pose.orientation.z = q[2]
#             msg.pose.orientation.w = q[3]
            
#             self.goal_pub.publish(msg)

# def main():
#     rclpy.init()
#     time.sleep(3)
#     node = PathPlannerNode()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



