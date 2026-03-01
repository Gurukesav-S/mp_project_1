import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from std_msgs.msg import Int32
from vrx_navigation.search_algorithms import a_star 
from scipy.spatial.transform import Rotation as R
import time

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        
        self.goal_pos = (105, 75) # (x, y)
        self.current_pos = None
        self.current_yaw = 0.0
        self.grid = None
        self.cell_size = 10.0
        self.path = []
        self.current_wp_idx = 0
        
        self.total_expanded_nodes = 0

        # --- NEW: State Tracking Flags ---
        self.map_changed = False
        self.needs_replanning = True # Start true so it plans the first route

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

        # --- NEW: Only react if the map actually changed structurally ---
        if self.grid is None or not np.array_equal(self.grid, new_grid):
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

    def guidance_loop(self):
        if self.grid is None or self.current_pos is None:
            return

        # 1. Map Update Check
        if self.map_changed:
            self.map_changed = False # Consume flag
            
            # If we have a path, verify it's not blocked
            if self.path and self.current_wp_idx < len(self.path):
                path_blocked = False
                for i in range(self.current_wp_idx, len(self.path)):
                    wx, wy = self.path[i]
                    r, c = self.world_to_grid(wx, wy)
                    if 0 <= r < self.grid.shape[0] and 0 <= c < self.grid.shape[1]:
                        if self.grid[r, c] == 1:
                            path_blocked = True
                            break
                
                if path_blocked:
                    self.get_logger().info("Path blocked! Clearing path.")
                    self.path = [] 
                    self.current_wp_idx = 0
                    
                    empty_path = Path()
                    empty_path.header.stamp = self.get_clock().now().to_msg()
                    empty_path.header.frame_id = 'odom'
                    self.path_pub.publish(empty_path)
            
            # If we have NO path (either blocked or A* previously failed), request a replan
            if not self.path:
                self.needs_replanning = True

        # 2. Plan path ONLY if the replanning flag is True
        if self.needs_replanning:
            self.needs_replanning = False # Consume flag so we ONLY try once!
            
            r, c = self.world_to_grid(*self.current_pos)
            r = max(0, min(r, self.grid.shape[0]-1))
            c = max(0, min(c, self.grid.shape[1]-1))
            start_grid = (r, c)
            goal_grid = self.world_to_grid(*self.goal_pos)
            
            if self.grid[start_grid[0], start_grid[1]] == 1 or self.grid[goal_grid[0], goal_grid[1]] == 1:
                self.get_logger().warn("Start or Goal is blocked. Standing by...")
                self.hold_position()
                return

            dummy_costs = np.ones_like(self.grid)
            self.get_logger().info("Running A* Planner...")
            grid_path, expanded_count = a_star(self.grid, start_grid, goal_grid, dummy_costs, 0)
            
            self.total_expanded_nodes += expanded_count
            exp_msg = Int32()
            exp_msg.data = self.total_expanded_nodes
            self.expanded_pub.publish(exp_msg)
            
            if grid_path:
                self.path = [self.grid_to_world(r, c) for r, c in grid_path]
                self.get_logger().info(f"Path Planned: {len(self.path)} waypoints. Total Expanded: {self.total_expanded_nodes}")
                
                path_msg = Path()
                path_msg.header.stamp = self.get_clock().now().to_msg()
                path_msg.header.frame_id = 'odom'
                for wx, wy in self.path:
                    pose = PoseStamped()
                    pose.pose.position.x = float(wx)
                    pose.pose.position.y = float(wy)
                    path_msg.poses.append(pose)
                self.path_pub.publish(path_msg)
            else:
                self.get_logger().warn("No path found! Waiting for map to change...")
                self.hold_position()
                return # Give up for now, wait for the map to change again!

        # 3. LOS / Waypoint following logic
        if self.path and self.current_wp_idx < len(self.path):
            target_x, target_y = self.path[self.current_wp_idx]
            
            dist = np.hypot(target_x - self.current_pos[0], target_y - self.current_pos[1])
            if dist < 3.0:
                self.current_wp_idx += 1
                return

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.pose.position.x = target_x
            msg.pose.position.y = target_y
            
            bearing = np.arctan2(target_y - self.current_pos[1], target_x - self.current_pos[0])
            q = R.from_euler('z', bearing).as_quat()
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]
            
            self.goal_pub.publish(msg)

def main():
    rclpy.init()
    time.sleep(3)
    node = PathPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



