#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class DriveReplay(Node):
    def __init__(self):
        super().__init__('drive_replay')
        from rclpy.exceptions import ParameterAlreadyDeclaredException
        try:
            self.declare_parameter('use_sim_time', True)
        except ParameterAlreadyDeclaredException:
            pass

        self.get_logger().info('DriveReplay node indítva, use_sim_time = True')

        # 1) Feliratkozunk a bag-play által publikált ackermann_cmd-re
        self.sub = self.create_subscription(
            AckermannDriveStamped,
            '/ackermann_cmd',
            self.on_drive_cmd,
            10)

        # 2) Létrehozzuk a publisher-t, ami továbbítja ugyanazt a parancsot
        self.pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)

    def on_drive_cmd(self, msg: AckermannDriveStamped):
        # egyszerű továbbküldés
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriveReplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
#       rclpy.shutdown() --> ez nem kell, mert a launch fájlban leállítja a rosbag_play

if __name__ == '__main__':
    main()
