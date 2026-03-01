import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from std_msgs.msg import Int32
from vrx_navigation.search_algorithms import a_star 
from scipy.spatial.transform import Rotation as R

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        
        # Hardcoded Goal in ENU meters
        self.goal_pos = (105, 75) # (x, y)
        self.current_pos = None
        self.grid = None
        self.cell_size = 10.0
        self.path = []
        self.current_wp_idx = 0

        # Subscriptions
        self.create_subscription(OccupancyGrid, '/map_grid', self.map_callback, 10)
        self.create_subscription(Odometry, '/kf/odom', self.odom_callback, 10)
        
        # Publisher to the PID controller
        self.goal_pub = self.create_publisher(PoseStamped, '/pid/goal', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.expanded_pub = self.create_publisher(Int32, '/expanded_nodes', 10) # NEW PUBLISHER
        # Timer for LOS logic
        self.create_timer(0.2, self.guidance_loop)

    def map_callback(self, msg):
        # Convert OccupancyGrid back to 2D numpy array (1 for obstacle, 0 for free)
        rows = msg.info.height
        cols = msg.info.width
        self.cell_size = msg.info.resolution
        self.grid = np.array(msg.data).reshape((rows, cols))
        self.grid = (self.grid > 50).astype(int) # Thresholding to binary

    def odom_callback(self, msg):
        self.current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def world_to_grid(self, x, y):
        return int(y // self.cell_size), int(x // self.cell_size)

    def grid_to_world(self, row, col):
        # Center of the cell
        return col * self.cell_size + (self.cell_size/2), row * self.cell_size + (self.cell_size/2)

    def guidance_loop(self):
        if self.grid is None or self.current_pos is None:
            return

        # 1. Plan path if not already planned
        if not self.path:
            start_grid = (0,0) #self.world_to_grid(*self.current_pos)
            goal_grid = self.world_to_grid(*self.goal_pos)
            
            # Use your A* logic (heuristic=0 for Manhattan)
            # Costs array of 1s as requested
            dummy_costs = np.ones_like(self.grid)
            grid_path, expanded_count = a_star(self.grid, start_grid, goal_grid, dummy_costs, 0)
            
            if grid_path:
                self.path = [self.grid_to_world(r, c) for r, c in grid_path]
                self.get_logger().info(f"Path Planned: {len(self.path)} waypoints.")

                exp_msg = Int32()
                exp_msg.data = expanded_count
                self.expanded_pub.publish(exp_msg)
                
                path_msg = Path()
                path_msg.header.stamp = self.get_clock().now().to_msg()
                path_msg.header.frame_id = 'odom'
                
                for wx, wy in self.path:
                    pose = PoseStamped()
                    pose.pose.position.x = float(wx)
                    pose.pose.position.y = float(wy)
                    path_msg.poses.append(pose)
                
                self.path_pub.publish(path_msg)

        # 2. LOS / Waypoint following logic
        if self.path and self.current_wp_idx < len(self.path):
            target_x, target_y = self.path[self.current_wp_idx]
            
            # Check if reached current waypoint (radius of 4m as per your LOS_vrx.py)
            dist = np.hypot(target_x - self.current_pos[0], target_y - self.current_pos[1])
            if dist < 3.0:
                self.current_wp_idx += 1
                return

            # Publish target to PID
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.pose.position.x = target_x
            msg.pose.position.y = target_y
            
            # Calculate heading toward waypoint for the PID
            bearing = np.arctan2(target_y - self.current_pos[1], target_x - self.current_pos[0])
            q = R.from_euler('z', bearing).as_quat()
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
            
            self.goal_pub.publish(msg)

def main():
    rclpy.init()
    node = PathPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()