import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from scipy.spatial.transform import Rotation as R
import time
import numpy as np
import math
from vrx_navigation.search_algorithms import ImprovedDLite

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        self.goal_pos = (155.0, 135.0) 
        self.R_min = 3.0  # USV Minimum Turning Radius [cite: 470]
        
        self.current_pos = None
        self.current_yaw = 0.0
        self.grid = None
        self.map_origin = (0.0, 0.0)
        self.cell_size = 1.0
        self.planner = None
        self.last_goal = None

        self.create_subscription(OccupancyGrid, '/map_grid', self.map_callback, 10)
        self.create_subscription(Odometry, '/kf/odom', self.odom_callback, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/pid/goal', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.create_timer(1.0, self.guidance_loop)

    def map_callback(self, msg):
        self.cell_size = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.grid = (self.grid > 50).astype(int)

    def odom_callback(self, msg):
        self.current_pos = (msg.pose.pose.position.x, 
                            msg.pose.pose.position.y)

    def _find_nearest_safe_world_point(self, x, y, search_radius=5):
        """
        Checks if (x,y) is safe. If it hits an obstacle, searches outward 
        in a grid pattern to find and return the nearest safe world coordinate.
        """
        if self.grid is None:
            return x, y

        r_center, c_center = self.world_to_grid(x, y)
        
        # 1. If it's already safe, return the original point exactly as-is
        if 0 <= r_center < self.grid.shape[0] and 0 <= c_center < self.grid.shape[1]:
            if self.grid[r_center, c_center] == 0:
                return x, y
                
        # 2. If it hit an obstacle, search for the closest safe cell
        min_dist = float('inf')
        best_r, best_c = r_center, c_center
        found_safe = False
        
        # Search a box around the trapped point
        for dr in range(-search_radius, search_radius + 1):
            for dc in range(-search_radius, search_radius + 1):
                r = r_center + dr
                c = c_center + dc
                
                # Check bounds
                if 0 <= r < self.grid.shape[0] and 0 <= c < self.grid.shape[1]:
                    # If we found free water
                    if self.grid[r, c] == 0: 
                        # Calculate true distance to find the absolute closest safe cell
                        dist = math.hypot(dr, dc)
                        if dist < min_dist:
                            min_dist = dist
                            best_r, best_c = r, c
                            found_safe = True
                            
        # 3. Snap the coordinate to the center of the nearest safe grid cell
        if found_safe:
            return self.grid_to_world(best_r, best_c)
            
        # Fallback if completely surrounded (unlikely if A* succeeded)
        return None
    
    def apply_idw_interpolation(self, world_path):
        """Shift points to grid boundaries, pushing outward if hitting obstacles."""
        if len(world_path) < 3: return world_path
        new_path = [world_path[0]]
        
        for i in range(1, len(world_path)-1):
            p_prev = np.array(world_path[i-1])
            p_curr = np.array(world_path[i])
            p_next = np.array(world_path[i+1])
            
            # IDW shift
            weight_curr = 0.7
            weight_adj = (1 - weight_curr) / 2
            interpolated = p_curr * weight_curr + p_prev * weight_adj + p_next * weight_adj
            
            # Shift the point to the nearest safe cell if the interpolation dragged it into a wall
            safe_pt = self._find_nearest_safe_world_point(interpolated[0], interpolated[1])
            
            if safe_pt:
                new_path.append(safe_pt)
            else:
                new_path.append(tuple(p_curr)) # Extreme fallback
                
        new_path.append(world_path[-1])
        return new_path

    def apply_dubins_smoothing(self, path):
        """Generates linear segments, bending the curve around obstacles."""
        if len(path) < 2: return path
        smooth_pts = []
        
        for i in range(len(path)-1):
            p1, p2 = np.array(path[i]), np.array(path[i+1])
            steps = max(int(np.linalg.norm(p2-p1) / 0.5), 1)
            
            for t in np.linspace(0, 1, steps, endpoint=False):
                pt = p1 * (1-t) + p2 * t
                
                # Automatically push any straight-line segments that clip a wall outward
                safe_pt = self._find_nearest_safe_world_point(pt[0], pt[1])
                
                if safe_pt:
                    smooth_pts.append(safe_pt)
                else:
                    smooth_pts.append(tuple(pt))
                
        smooth_pts.append(tuple(path[-1]))
        return smooth_pts

    def guidance_loop(self):
        if self.grid is None or self.current_pos is None: return
        start_grid = self.world_to_grid(self.current_pos[0], self.current_pos[1])
        goal_grid = self.world_to_grid(self.goal_pos[0], self.goal_pos[1])

        if math.hypot(self.goal_pos[0]-self.current_pos[0], self.goal_pos[1]-self.current_pos[1]) < 2.0: return

        if self.planner is None or self.last_goal != self.goal_pos:
            self.planner = ImprovedDLite(self.grid, start_grid, goal_grid)
            self.last_goal = self.goal_pos

        grid_path, _ = self.planner.extract_path(start_grid)
        if grid_path:
            world_path = [self.grid_to_world(r, c) for r, c in grid_path]
            # Post-processing sequence from paper [cite: 384, 385]
            idw_path = self.apply_idw_interpolation(world_path)
            final_path = self.apply_dubins_smoothing(idw_path)
            
            self.publish_path(final_path)
            target = final_path[min(2, len(final_path)-1)] # Lookahead
            self.send_to_pid(target[0], target[1])

    def world_to_grid(self, x, y):
        # Calculate raw indices
        c = int((x - self.map_origin[0]) / self.cell_size)
        r = int((y - self.map_origin[1]) / self.cell_size)
        
        # Get max bounds based on your current map size
        max_r = self.grid.shape[0] - 1
        max_c = self.grid.shape[1] - 1
        
        # CLAMP: Force the coordinates to stay within 0 and the max grid size
        safe_r = max(0, min(r, max_r))
        safe_c = max(0, min(c, max_c))
        
        return (safe_r, safe_c)
    
    def grid_to_world(self, row, col):
        return col * self.cell_size + (self.cell_size/2), row * self.cell_size + (self.cell_size/2)

    def send_to_pid(self, x, y):
        msg = PoseStamped()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), 'map'
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        self.goal_pub.publish(msg)

    def publish_path(self, points):
        msg = Path()
        msg.header.frame_id = 'map'
        for pt in points:
            p = PoseStamped()
            p.pose.position.x, p.pose.position.y = pt[0], pt[1]
            msg.poses.append(p)
        self.path_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()