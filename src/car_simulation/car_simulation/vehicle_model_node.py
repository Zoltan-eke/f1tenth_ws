#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster

class VehicleModelNode(Node):
    def __init__(self):
        super().__init__('vehicle_model_node')

        # --- parameters ---
        self.declare_parameter('mass',             1845.0)   # [kg]
        self.declare_parameter('Iz',               2500.0)   # [kg·m²]
        self.declare_parameter('Cf',               4000.0)   # [N/rad]
        self.declare_parameter('Cr',               4000.0)   # [N/rad]
        self.declare_parameter('lf',               1.4375)   # [m]
        self.declare_parameter('lr',               1.4375)   # [m]
        self.declare_parameter('tau_velocity',     0.2)     # [s]
        self.declare_parameter('Cd',               0.3)      # drag coefficient
        self.declare_parameter('A',                2.2)      # frontal area [m²]
        self.declare_parameter('max_speed',        30.0)     # fizikai sebességkorlát [m/s]

        self._m     = self.get_parameter('mass').value
        self._Iz    = self.get_parameter('Iz').value
        self._Cf    = self.get_parameter('Cf').value
        self._Cr    = self.get_parameter('Cr').value
        self._lf    = self.get_parameter('lf').value
        self._lr    = self.get_parameter('lr').value
        self._tau   = self.get_parameter('tau_velocity').value
        self._Cd    = self.get_parameter('Cd').value
        self._A     = self.get_parameter('A').value
        self._rho   = 1.225  # kg/m³
        self._v_max = self.get_parameter('max_speed').value

        # --- state variables ---
        self._x     = 0.0    # global x position
        self._y     = 0.0    # global y position
        self._theta = 0.0    # yaw angle
        self._v     = 0.0    # longitudinal speed
        self._vy    = 0.0    # lateral speed
        self._r     = 0.0    # yaw rate
        self._last_time = None
        self._pose_initialized = False
        self._has_logged_start = False

        # --- ROS interfészek ---
        self._odom_pub = self.create_publisher(Odometry, 'follower_odom', 10)

        self._sub      = self.create_subscription(
            AckermannDriveStamped,
            '/carla/ego_vehicle/ackermann_cmd',
            self._cmd_callback,
            10
        )
        self._init_sub = self.create_subscription(
            PoseStamped,
            '/initial_pose',
            self._init_pose_callback,
            10
        )

    def _init_pose_callback(self, msg: PoseStamped):
        if not self._pose_initialized:
            self._x = msg.pose.position.x
            self._y = msg.pose.position.y
            self._theta = self._yaw_from_quaternion(msg.pose.orientation)
            self._pose_initialized = True
            self.get_logger().info(f"Kezdő pozíció beállítva: x={self._x:.2f}, y={self._y:.2f}, yaw={self._theta:.2f}")

    def _yaw_from_quaternion(self, q):
        return math.atan2(2.0*(q.w*q.z + q.x*q.y),
                          1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def _derivatives(self, state, v_cmd, delta):
        """
        Compute state derivatives for the dynamic bicycle + drag + lag model.
        state = [x, y, theta, v, vy, r]
        """
        x, y, theta, v, vy, r = state
        # clamp-olt v draghoz
        v_clamped = max(0.0, min(v, self._v_max))
        F_drag    = 0.5 * self._rho * self._Cd * self._A * v_clamped**2
        # first-order sebességlag
        a_cmd = (v_cmd - v) / self._tau
        a_net = a_cmd - F_drag / self._m
        dv    = a_net

        # slip‐szögek, oldalerők
        if abs(v) > 1e-3:
            alpha_f = math.atan2(vy + self._lf * r, v) - delta
            alpha_r = math.atan2(vy - self._lr * r, v)
        else:
            alpha_f = alpha_r = 0.0

        F_yf = -self._Cf * alpha_f
        F_yr = -self._Cr * alpha_r

        # yaw rate and lateral speed derivatives
        dr  = (self._lf * F_yf - self._lr * F_yr) / self._Iz
        dvy = (F_yf + F_yr) / self._m - v * r

        # kinematic derivatives
        dx     = v * math.cos(theta) - vy * math.sin(theta)
        dy     = v * math.sin(theta) + vy * math.cos(theta)
        dtheta = r

        return [dx, dy, dtheta, dv, dvy, dr]

    def _rk4_step(self, state, v_cmd, delta, dt):
        """Perform a single RK4 integration step."""
        k1 = self._derivatives(state,              v_cmd, delta)
        s2 = [s + 0.5*dt*k for s, k in zip(state, k1)]
        k2 = self._derivatives(s2,                 v_cmd, delta)
        s3 = [s + 0.5*dt*k for s, k in zip(state, k2)]
        k3 = self._derivatives(s3,                 v_cmd, delta)
        s4 = [s +     dt*k for s, k in zip(state, k3)]
        k4 = self._derivatives(s4,                 v_cmd, delta)

        new_state = [
            s + (dt/6.0)*(k1_i + 2*k2_i + 2*k3_i + k4_i)
            for s, k1_i, k2_i, k3_i, k4_i
            in zip(state, k1, k2, k3, k4)
        ]
        # clamp-olt hosszanti sebesség
        new_state[3] = max(0.0, min(self._v_max, new_state[3]))
        return new_state

    def _cmd_callback(self, msg: AckermannDriveStamped):
        if not self._pose_initialized:
            return
        
        if not self._has_logged_start:
            self.get_logger().info('Szimuláció elindult')
            self._has_logged_start = True
            
        # compute dt from simulation time stamp
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_time is None:
            self._last_time = t
            return
        dt = t - self._last_time
        self._last_time = t
        if dt <= 0.0:
            return

        # clamp-olt parancssebesség
        v_cmd = max(0.0, min(msg.drive.speed, self._v_max))
        delta = msg.drive.steering_angle

        # RK4 integráció
        state     = [self._x, self._y, self._theta, self._v, self._vy, self._r]
        new_state = self._rk4_step(state, v_cmd, delta, dt)
        self._x, self._y, self._theta, self._v, self._vy, self._r = new_state

        # --- publish odometry ---
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = 'follower_odom'
        odom.child_frame_id = 'follower_base_link'
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0

         # quaternion-normalizálás
        q = self._quaternion_from_euler(0.0, 0.0, self._theta)
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        q.x /= norm; q.y /= norm; q.z /= norm; q.w /= norm
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x  = self._v
        odom.twist.twist.linear.y  = self._vy
        odom.twist.twist.linear.z  = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self._r

        self._odom_pub.publish(odom)

    @staticmethod
    def _quaternion_from_euler(roll, pitch, yaw):
        q = Quaternion()
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        q.w = cy * cr * cp + sy * sr * sp
        q.x = cy * sr * cp - sy * cr * sp
        q.y = cy * cr * sp + sy * sr * cp
        q.z = sy * cr * cp - cy * sr * sp
        return q

def main(args=None):
    rclpy.init(args=args)
    node = VehicleModelNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()