#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
import math

class OpenLoopCmdPublisher(Node):
    def __init__(self):
        super().__init__('open_loop_cmd_publisher')
        # Paraméterek
        self.declare_parameter('wheel_base', 2.875)
        self.declare_parameter('max_speed', 30.0)
        self.declare_parameter('max_steer', 0.5)


        self.wb        = self.get_parameter('wheel_base').value
        self.v_max     = self.get_parameter('max_speed').value
        self.steer_max = self.get_parameter('max_steer').value

        # Belső állapot
        self.prev_pose    = None
        self.prev_time    = None
        self.prev_heading = None

        # ROS interfészek
        self.leader_sub = self.create_subscription(
            PoseStamped,
            'initial_pose',
            self.pose_cb,
            10
        )
        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped,
            '/carla/ego_vehicle/ackermann_cmd',
            10
        )
        # 10 Hz-es timer, így lesz konstans frissítés
        self.timer = self.create_timer(0.1, self.timer_cb)

        self.get_logger().info('OpenLoopCmdPublisher initialized.')

    def pose_cb(self, msg: PoseStamped):
        # Csak tároljuk a legutóbbi leader_pose-ot
        self.latest_pose = msg

    def timer_cb(self):
        # Ha még nincs pose, nincs mit csinálni
        if not hasattr(self, 'latest_pose'):
            return

        msg = self.latest_pose
        # 1) Időbélyeg másodpercben
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # 2) Első futás inicializálás
        if self.prev_time is None:
            self.prev_time = t
            self.prev_pose = msg
            return

        # 3) dt kiszámítása
        dt = t - self.prev_time
        if dt <= 0.0:
            return

        # 4) Pozíció-különbség és sebesség
        p     = msg.pose.position
        q_prev = self.prev_pose.pose.position
        dx    = p.x - q_prev.x
        dy    = p.y - q_prev.y
        dist  = math.hypot(dx, dy)
        speed = dist / dt

        # 5) Heading és curvature
        heading = math.atan2(dy, dx)
        if self.prev_heading is not None and dist > 1e-3:
            dh = (heading - self.prev_heading + math.pi) % (2*math.pi) - math.pi
            curvature = dh / dist
        else:
            curvature = 0.0

        steer = math.atan(self.wb * curvature)

        # 6) Clamp a fizikai maximumokra
        v_cmd     = max(0.0, min(speed, self.v_max))
        steer_cmd = max(-self.steer_max, min(self.steer_max, steer))

        # 7) Publikáljuk az Ackermann Drive-ot
        cmd = AckermannDriveStamped()
        cmd.header.stamp = msg.header.stamp
        cmd.drive.speed = v_cmd
        cmd.drive.steering_angle = steer_cmd
        self.cmd_pub.publish(cmd)

        # 8) Állapotfrissítés a következő iterációhoz
        self.prev_time    = t
        self.prev_pose    = msg
        self.prev_heading = heading


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopCmdPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
