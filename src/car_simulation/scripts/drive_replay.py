#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class DriveReplay(Node):
    def __init__(self):
        super().__init__('drive_replay')
        # sim-time engedélyezése
        # self.declare_parameter('use_sim_time', True)

        # A bag-be rögzített /drive topic feliratkozása
        self.sub = self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.drive_callback,
            10
        )
        # /ackermann_cmd újraküldése
        self.pub = self.create_publisher(
            AckermannDriveStamped,
            '/ackermann_replay_cmd',
            10
        )
        self.get_logger().info('DriveReplay node indítva, hallgat /drive-ra, publikál /ackermann_cmd-re')

    def drive_callback(self, msg: AckermannDriveStamped):
        # Frissítsd a fejléc időbélyegét, ha szeretnéd:
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DriveReplay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
