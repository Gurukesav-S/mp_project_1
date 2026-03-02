import rclpy
from rclpy.node import Node
import signal
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool 
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point, Quaternion
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
class LOS_vrx(Node):

    def __init__(self):
        super().__init__('LOS_node')
        signal.signal(signal.SIGINT, self.handle_signal)   
        self.waypoints = np.array([[75.0,5.0,1.5708],[75.0,45.0,1.5708],[5.0,45.0,0.7854],[75.0,45.0,1.5708],[10.0,25.0,2.3562],])
        #self.waypoints = np.array([])
        self.msg_pid = PoseStamped()
        self.create_subscription(Odometry, '/kf/odom', self.loc_callback, 10)
        self.pub_goal_pid = self.create_publisher(PoseStamped, '/pid/goal', 10)
        self.pub_in_final_goal = self.create_publisher(Bool, '/finalgoal_reached', 10)
        self.create_timer(0.1, self.goal_publisher)
        self.i = 0
        self.first_wp_flag = True
        self.GCS_x = self.GCS_y = self.los_x = self.los_y = self.apf_waypoint_x = self.apf_waypoint_y = None
        self.first_wp_flag = True
        self.yaw_tol = np.deg2rad(360)
        self.wp_rad = 4.0
        self.finalgoal = np.array([150.0,100.0])
        self.in_final_goal = False
        self.finalgoal_reached = False
        

    def handle_signal(self, signum, frame):
        rclpy.shutdown()    
    
    def loc_callback(self, msg):
        self.GCS_x, self.GCS_y = msg.pose.pose.position.x, msg.pose.pose.position.y
        orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.GCS_yaw = R.from_quat(orientation).as_euler('xyz')[2]

    def LOS(self, prev_wp, current_wp, current_pos):
            x1, y1, yaw1, x2, y2, yaw2, x3, y3, yaw3 = *prev_wp, *current_wp, *current_pos
            dist = np.linalg.norm(current_pos - current_wp)
            l = ((x2 - x1) * (x3 - x1) + (y2 - y1) * (y3 - y1)) / ((x2 - x1) ** 2 + (y2 - y1) ** 2)
            x4, y4 = l * (x2 - x1) + x1, l * (y2 - y1) + y1
            d2 = np.linalg.norm([x4 - x2, y4 - y2])
            delta = max(min(5.0, dist), 3.0)
            x5, y5 = ((d2 - delta) * x4 + delta * x2) / d2, ((d2 - delta) * y4 + delta * y2) / d2
            self.los_x, self.los_y = x5, y5
            
            if dist < 5.0:
                x5,y5=x2,y2
            yaw_desired = yaw2
            self.msg_pid.pose.position.x, self.msg_pid.pose.position.y, self.msg_pid.pose.position.z = x5, y5, 0.0
            quat = R.from_euler('xyz', [0, 0, yaw_desired]).as_quat()
            self.msg_pid.header.frame_id = 'odom'
            self.msg_pid.pose.orientation.x, self.msg_pid.pose.orientation.y, self.msg_pid.pose.orientation.z, self.msg_pid.pose.orientation.w = quat
            return self.msg_pid
    
    def goal_publisher(self):
        if self.GCS_x is not None:
            if self.finalgoal.size:
                current_pos = np.array([self.GCS_x, self.GCS_y, self.GCS_yaw])

                if self.first_wp_flag:
                    #self.first_wp = current_pos
                    self.first_wp = np.array([5.0,5.0,0.0])
                    self.first_wp_flag = False

                prev_wp = self.first_wp if self.i == 0 else self.waypoints[self.i - 1]

                if self.i < len(self.waypoints):
                    current_wp = self.waypoints[self.i]
                else:
                    last_yaw = self.waypoints[-1][2]
                    current_wp = np.append(self.finalgoal, last_yaw)
                    self.in_final_goal = True
                x1, y1, yaw1, x2, y2, yaw2, x3, y3, yaw3 = *prev_wp, *current_wp, *current_pos
                if y2!=y1: z=((y3-y2) +(((x2-x1)/(y2-y1))*(x3-x2)))
                pos_err = np.linalg.norm(current_pos[:2] - current_wp[:2])
                yaw_err = ((current_pos[2] - current_wp[2] + np.pi) % (2 * np.pi)) - np.pi
                self.get_logger().info(f"pos_err:{pos_err}, yaw_err:{yaw_err}, yaw_tol:{self.yaw_tol}")
                if pos_err < self.wp_rad : #and abs(yaw_err) < self.yaw_tol or -1.0<(z)<1.0: #or -1.0<(z)<1.0
                    if self.in_final_goal:
                        self.get_logger().info(f"Final goal reached")
                        self.finalgoal_reached = True
                        self.msg_pid.pose.position.x, self.msg_pid.pose.position.y, self.msg_pid.pose.position.z = current_wp
                        quat = R.from_euler('xyz', [0, 0, last_yaw]).as_quat()
                        self.msg_pid.header.frame_id = 'odom'
                        self.msg_pid.pose.orientation.x, self.msg_pid.pose.orientation.y, self.msg_pid.pose.orientation.z, self.msg_pid.pose.orientation.w = quat
                        self.pub_goal_pid.publish(self.msg_pid)
                    else:    
                        self.get_logger().info(f"Waypoint {self.i} reached")
                        self.i += 1
                    #if (current_wp[:2] != self.finalgoal).all():
                        # self.get_logger().info(f"Waypoint {self.i} reached")
                        # self.i += 1
                else:
                    self.msg_pid = self.LOS(prev_wp, current_wp, current_pos)
                    self.pub_goal_pid.publish(self.msg_pid)

                bool_msg = Bool()
                bool_msg.data = self.finalgoal_reached
                self.pub_in_final_goal.publish(bool_msg)
            # else:
            #     goal_lat = float(input("Enter finalgoal lat (deg): "))
            #     goal_long = float(input("Enter finalgoal long (deg): "))
            #     self.finalgoal = gps_to_xy_ENU(goal_long, goal_lat)
        else:
            print('!kf')
        
def main(args=None):
    rclpy.init(args=args)
    LOS_node = LOS_vrx()
    try:
        #time.sleep(2)
        while rclpy.ok():
            rclpy.spin_once(LOS_node)
            # guidance_node.plotter.plot_current_state(guidance_node.los_x, guidance_node.los_y, guidance_node.apf_waypoint_x, guidance_node.apf_waypoint_y, guidance_node.GCS_x, guidance_node.GCS_y, guidance_node.waypoints, guidance_node.finalgoal, guidance_node.buoy_array, guidance_node.obstacles, guidance_node.wp_rad, guidance_node.rho_ob)
    except KeyboardInterrupt:
        LOS_node.destroy_node()

if __name__ == '__main__':
    main()
