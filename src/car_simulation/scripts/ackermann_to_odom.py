#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

class AckermannToOdom(Node):
    def __init__(self):
        super().__init__('ackermann_to_odom')

        # sim-time engedélyezése
        #from rclpy.exceptions import ParameterAlreadyDeclaredException
        #try:
        #    self.declare_parameter('use_sim_time', True)
        #except ParameterAlreadyDeclaredException:
        #    pass

        # csak a wheel_base paramétert deklaráljuk
        self.declare_parameter('wheel_base', 0.320)
        self._wb = self.get_parameter('wheel_base').get_parameter_value().double_value

        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        # itt rclpy Time objektumot tartunk
        self._last_time = None

        # feliratkozunk az ackermann üzenetekre
        self.create_subscription(
            AckermannDriveStamped,
            '/ackermann_cmd',
            self._cmd_callback,
            10
        )
        # ide publikáljuk (launch-ból remappelve lesz sim_odom)
        self._pub = self.create_publisher(Odometry, '/odom', 10)

        self.get_logger().info(f"AckToOdom indítva, wheel_base={self._wb}")

    def _euler_to_quaternion(self, yaw: float) -> Quaternion:
        half = yaw * 0.5
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(half)
        q.w = math.cos(half)
        return q

    def _cmd_callback(self, msg: AckermannDriveStamped):
        # vegyük az üzenet fejléc-időbélyegét
        now = self.get_clock().now()
        if self._last_time is None:
            self._last_time = now
            return
        
        # dt a két callback között (sec)
        dt = (now.nanoseconds - self._last_time.nanoseconds) * 1e-9
        self._last_time = now

        v     = msg.drive.speed
        delta = msg.drive.steering_angle

        # Euler-integrálás
        self._x     += v * dt * math.cos(self._theta)
        self._y     += v * dt * math.sin(self._theta)
        self._theta += v * dt * math.tan(delta) / self._wb

        # összeállítjuk az Odometry-t a bag időbélyegével
        odom = Odometry()
        # most a sim-time szerint tesszük a stamp-et
        odom.header.stamp    =  now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x    = self._x
        odom.pose.pose.position.y    = self._y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation    = self._euler_to_quaternion(self._theta)

        odom.twist.twist.linear.x    = v
        odom.twist.twist.angular.z   = v * math.tan(delta) / self._wb

        self._pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = AckermannToOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()