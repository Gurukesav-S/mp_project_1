import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from  std_msgs.msg import Int32MultiArray, Float64, Bool
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
import signal

class PID(Node):
    def __init__(self) -> None:
        super().__init__('pid_stationkeep')
        signal.signal(signal.SIGINT, self.handle_signal)

        # Goal and current state
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None
        self.GCS_x = None
        self.GCS_y = None
        self.GCS_yaw = None
        self.oe = None
        self.finalgoal_reached = False
       
        # Timer for dt
        self.prev_time = time.time()
        
        self.lever =  1.0 # half breadth

        # PID state variables
        self.prev_dist_err = 0.0
        self.int_dist_err = 0.0

        self.prev_ang_err = 0.0
        self.int_ang_err = 0.0

        self.prev_ori_err = 0.0
        self.int_ori_err = 0.0

        # Distance thresholds (meters)
        self.th1 = 5.0   # far threshold
        self.th2 = 1.0   # close threshold

        self.create_subscription(Odometry, '/kf/odom', self.loc_callback, 10)
        self.create_subscription(PoseStamped, '/pid/goal', self.goal_enu_callback, 20)
        self.create_subscription(Bool, '/finalgoal_reached', self.final_goal_callback, 10)
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.thruster_publisher_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.thruster_publisher_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        #self.pub_pid_goal = self.create_publisher(Int32MultiArray, '/uros_ardudue/thruster_subscriber', 10)

    def handle_signal(self, signum, frame):
        self.thruster_publisher_left.publish(0.0)
        self.thruster_publisher_right.publish(0.0)   

        
        rclpy.shutdown()

    def goal_enu_callback(self, goal):
        self.goal_x = goal.pose.position.x
        self.goal_y = goal.pose.position.y
        goal_orientation = [goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w]
        self.goal_yaw = R.from_quat(goal_orientation).as_euler('xyz')[2]
        self.get_logger().info(f"LOS got goal: x={self.goal_x:.2f}, y={self.goal_y:.2f} yaw:{self.goal_yaw:.2f}")
        
    def loc_callback(self, msg):
        self.GCS_x = msg.pose.pose.position.x
        self.GCS_y = msg.pose.pose.position.y
        GCS_orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.GCS_yaw = R.from_quat(GCS_orientation).as_euler('xyz')[2]
        self.get_logger().info(f"LOS got odom: x={self.GCS_x:.2f}, y={self.GCS_y:.2f}")
    
    def final_goal_callback(self, msg):
        # Update final goal flag
        self.finalgoal_reached = msg.data
        self.get_logger().info(f"Final goal flag: {self.finalgoal_reached}")

    def allocate_forces_to_thrusters(self, force_local_x, moment_local_z):
        TA = np.array([[1.0,1.0],  
                       [-self.lever,self.lever]])
                         
        F = np.array([[force_local_x], [moment_local_z]]) # local Forces and moments
        T = np.linalg.pinv(TA) @ F
        return T.ravel()
    
    # def force_to_pwm(self, thrust):
    #     if thrust < -0.05:
    #         pwm = 4.4913 * thrust**3 + 34.6510 * thrust**2 + 168.7654 * thrust + 1.4639e+03
    #         return max(pwm, 1325)
    #     elif thrust > 0.05:
    #         pwm = 2.1809 * thrust**3 + -22.0578 * thrust**2 + 134.6641 * thrust + 1.5363e+03
    #         return min(pwm, 1675)
    #     else:
    #         pwm = 1500
    #         return pwm

    def timer_callback(self):
        if self.goal_x != None and self.GCS_x != None:
            self.pid()

    def compute_forces(self,e_dist=0.0, e_ang=0.0, e_ori=0.0,kp_d=0.0, kd_d=0.0, ki_d=0.0,kp_a=0.0, kd_a=0.0, ki_a=0.0,kp_o=0.0, kd_o=0.0, ki_o=0.0):
        # Compute time delta
        now = time.time()
        dt = now - self.prev_time

        # Distance PID
        self.int_dist_err += e_dist * dt
        d_der = (e_dist - self.prev_dist_err)/dt if dt>0 else 0.0
        Fd = kp_d*e_dist + kd_d*d_der + ki_d*self.int_dist_err
        self.prev_dist_err = e_dist

        # Angle PID
        self.int_ang_err += e_ang * dt
        a_der = (e_ang - self.prev_ang_err)/dt if dt>0 else 0.0
        Mz_ang = kp_a*e_ang + kd_a*a_der + ki_a*self.int_ang_err
        self.prev_ang_err = e_ang

        # Orientation PID
        self.int_ori_err += e_ori * dt
        o_der = (e_ori - self.prev_ori_err)/dt if dt>0 else 0.0
        Mz_ori = kp_o*e_ori + kd_o*o_der + ki_o*self.int_ori_err
        self.prev_ori_err = e_ori

        # Save timestamp
        self.prev_time = now

        # Total thrust and moment
        return Fd, (Mz_ang + Mz_ori)
    
    def publish_thrust(self, left, right):
        msg_left = Float64()
        msg_right = Float64()
        msg_left.data = left
        msg_right.data = right
        self.thruster_publisher_left.publish(msg_left)
        self.thruster_publisher_right.publish(msg_right)   

    # PID function
    def pid(self):
        if self.finalgoal_reached:
            self.publish_thrust(0.0, 0.0)
            self.get_logger().info('In final goal: thrusters set to zero')
            return
        else:
            dx = np.around(self.goal_x - self.GCS_x, 2)
            dy = np.around(self.goal_y - self.GCS_y, 2)
            dist = np.hypot(dx, dy)
            bearing = np.arctan2(dy, dx)
            psi = self.GCS_yaw
            alpha = np.arctan2(np.sin(bearing - psi), np.cos(bearing - psi))

            Fd = 0.0
            Mz = 0.0
            heading_tolerance = np.radians(10)

            if abs(alpha) > heading_tolerance and dist > self.th2:
                self.get_logger().info(f"Aligning Heading: alpha={np.degrees(alpha):.2f}")
                # Use Alpha PID but keep Fd at 0.0
                _, Mz = self.compute_forces(
                    e_dist=0.0, e_ang=alpha, e_ori=0.0,
                    kp_a=15.0, kd_a=10.0, ki_a=0.0 # Slightly higher gains for spot-turn
                )
                Fd = 0.0

            elif dist > self.th1:
                Fd, Mz = self.compute_forces(
                    e_dist=dist, e_ang=alpha, e_ori=0.0,
                    kp_d=3.0, kd_d=0.1, ki_d=0.0,
                    kp_a=25.0, kd_a=10.0, ki_a=0.0
                )
                
            elif dist > self.th2:
                signed_dist = dist * np.cos(alpha) / abs(np.cos(alpha))
                Fd, Mz = self.compute_forces(
                    e_dist=signed_dist, e_ang=alpha, e_ori=0.0,
                    kp_d=3.0, kd_d=10.0, ki_d=0.0,
                    kp_a=25.0, kd_a=100.0, ki_a=0.0
                )
                
            else:
                ori_err = np.arctan2(np.sin(self.goal_yaw - psi), np.cos(self.goal_yaw - psi))
                self.oe= ori_err
                Fd, Mz = self.compute_forces(
                    e_dist=0.0, e_ang=0.0, e_ori=ori_err,
                    kp_o=50.0, kd_o=100.0, ki_o=0.0
                    )
                self.get_logger().info(f"ori_error:{self.oe}")

            thrusts = self.allocate_forces_to_thrusters(Fd, Mz)
            thrust_left = thrusts[0]*50
            thrust_right = thrusts[1]*50
            thrust_left = np.clip(thrust_left, -500.0, 500.0)
            thrust_right = np.clip(thrust_right, -500.0, 500.0)

            self.publish_thrust(thrust_left, thrust_right)

            self.get_logger().info(f"Thrust left :{thrust_left}| Thrust right: {thrust_right}")
            
def main(args=None):
    rclpy.init(args=args)
    pid1 = PID()
    try:
        rclpy.spin(pid1)
    except KeyboardInterrupt:
        pid1.destroy_node()

if __name__ == '__main__':
    main()