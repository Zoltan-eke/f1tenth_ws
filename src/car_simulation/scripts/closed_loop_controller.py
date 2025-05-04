#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class ClosedLoopController(Node):
    """
    Egyszerű zárt hurkú vezérlés:
    -- Feliratkozik a Carla ROS Bridge /carla/ego_vehicle/odometry topicra
    -- Kiad AckermannDriveStamped parancsot a /carla/ego_vehicle/ackermann_cmd-ra
    Alapértelmezett: fix (paraméterezhető) sebesség és nulla kormányzás.
    """

    def __init__(self):
        super().__init__('closed_loop_controller')

        # Parameter: célsebesség [m/s]
        self.declare_parameter('target_speed', 1.0)
        self.target_speed = self.get_parameter('target_speed').get_parameter_value().double_value

        # Publisher: AckermannDriveStamped
        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped,
            '/carla/ego_vehicle/ackermann_cmd',
            10
        )

        # Subscriber: Odometry (Carla bridge által szolgáltatott odom)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/carla/ego_vehicle/odometry',
            self.odom_callback,
            10
        )

        self.get_logger().info(f"ClosedLoopController elindult: target_speed={self.target_speed} m/s")

    def odom_callback(self, msg: Odometry):
        # Itt tehetsz bármilyen szabályozást:
        #   pl. pure pursuit, LQR, dinamikus bicikli modell előrejelzés stb.
        # Most csak konstans sebesség és nulla kormány:
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.drive.speed = float(self.target_speed)
        cmd.drive.steering_angle = 0.0

        self.cmd_pub.publish(cmd)
        self.get_logger().debug(f"Publikált parancs: v={cmd.drive.speed:.2f}, δ={cmd.drive.steering_angle:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
