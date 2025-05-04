#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

class VehicleModelNode(Node):
    def __init__(self):
        super().__init__('vehicle_model_node')

       # --- szimulációs idő ---
        try:
            self.declare_parameter('use_sim_time', True)
        except:
            pass

        # --- Paraméterek deklarálása ---
        self.declare_parameter('wheel_base', 0.32)
        self.declare_parameter('wheelbase_front', 0.063)
        self.declare_parameter('wheelbase_rear', 0.252)
        self.declare_parameter('Cf', 200.0)   # első gumierejő állandó [N/rad] 
        self.declare_parameter('Cr', 200.0)   # hátsó gumierejő állandó [N/rad] 
        self.declare_parameter('m', 3.74)     # tömeg [kg] 
        self.declare_parameter('Iz', 0.128)    # tehetetlenségi nyomaték [kg·m²]

        # Paraméterek betöltése
        self._wb  = self.get_parameter('wheel_base').value
        self._wbf = self.get_parameter('wheelbase_front').value
        self._wbr = self.get_parameter('wheelbase_rear').value
        self._Cf  = self.get_parameter('Cf').value
        self._Cr  = self.get_parameter('Cr').value
        self._m   = self.get_parameter('m').value
        self._Iz  = self.get_parameter('Iz').value

        # --- Állapotváltozók ---
        self._x     = 0.0   # globális X koordináta
        self._y     = 0.0   # globális Y koordináta
        self._theta = 0.0   # irány szög [rad]
        self._v     = 0.0   # hosszanti sebesség [m/s]
        self._r     = 0.0   # yaw-sebesség [rad/s]
        self._vy    = 0.0   # laterális sebesség [m/s]
        self._last_time = self.get_clock().now()

        # --- ROS interfészek ---
        self._odom_pub = self.create_publisher(Odometry, 'follower_odom', 10)
        self._tf_br    = TransformBroadcaster(self)
        self._sub      = self.create_subscription(
            AckermannDriveStamped,
            '/carla/ego_vehicle/ackermann_cmd',
            self._cmd_callback,
            10
        )


    def _cmd_callback(self, msg: AckermannDriveStamped):
        # Időlépés számítása
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        if dt <= 0.0:
            return

        # 1) parancsok kiolvasása
        self._v = msg.drive.speed
        delta = msg.drive.steering_angle

        # 2) slip-szögek számítása
        if abs(self._v) < 1e-3:
            alpha_f = 0.0
            alpha_r = 0.0
        else:
            alpha_f = delta - (self._vy + self._r * self._wbf) / self._v
            alpha_r =    - (self._vy - self._r * self._wbr) / self._v
        
        # 2a) SLIP-SZÖG SATURÁCIÓ
        alpha_f = max(-0.2, min(0.2, alpha_f))
        alpha_r = max(-0.2, min(0.2, alpha_r))

        # 3) oldalerők
        Fy_f = self._Cf * alpha_f
        Fy_r = self._Cr * alpha_r

        # 4) yaw-gyorsulás integrálása
        r_dot = (self._wbf * Fy_f - self._wbr * Fy_r) / self._Iz
        self._r += r_dot * dt

        # 5) laterális sebesség-derivált és integrálása
        vy_dot = (Fy_f + Fy_r) / self._m - self._r * self._v
        self._vy += vy_dot * dt

        # 6) pozíció és orientáció integrálása
        self._theta += self._r * dt
        self._x     += (self._v * math.cos(self._theta) - self._vy * math.sin(self._theta)) * dt
        self._y     += (self._v * math.sin(self._theta) + self._vy * math.cos(self._theta)) * dt

        # 7) Odometry üzenet összeállítása és publikálása
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'follower_odom'
        odom_msg.child_frame_id  = 'follower_base_link'
        odom_msg.pose.pose.position.x = self._x
        odom_msg.pose.pose.position.y = self._y
        odom_msg.pose.pose.position.z = 0.0
        quat = Quaternion()
        quat.z = math.sin(self._theta / 2.0)
        quat.w = math.cos(self._theta / 2.0)
        odom_msg.pose.pose.orientation = quat
        odom_msg.twist.twist.linear.x  = self._v
        odom_msg.twist.twist.linear.y  = self._vy
        odom_msg.twist.twist.angular.z = self._r
        self._odom_pub.publish(odom_msg)

        # 8) TF publikálása
        tfm = TransformStamped()
        tfm.header.stamp = now.to_msg()
        tfm.header.frame_id    = 'follower_odom'
        tfm.child_frame_id     = 'follower_base_link'
        tfm.transform.translation.x = self._x
        tfm.transform.translation.y = self._y
        tfm.transform.translation.z = 0.0
        tfm.transform.rotation      = quat
        self._tf_br.sendTransform(tfm)

    def destroy_node(self):
        super().destroy_node()

def main():
    rclpy.init()
    node = VehicleModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
