#!/usr/bin/env python3
import os
import csv
import sys

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from rosgraph_msgs.msg import Clock

class DriveReplay(Node):
    def __init__(self):
        super().__init__('drive_replay')

    # Paraméterek
        drive_file = self.declare_parameter('drive_file', '').value
        drive_topic = self.declare_parameter('topic', '/drive').value
        if not drive_file:
            self.get_logger().error("'drive_file' parameter is required")
            sys.exit(1)

        # CSV betöltése
        with open(drive_file, 'r') as f:
            reader = csv.DictReader(f)
            self._rows = [
                {
                    'time': float(row['time']),
                    'speed': float(row['speed']),
                    'steering_angle': float(row['steering_angle'])
                }
                for row in reader
            ]
        self._idx = 0
        if not self._rows:
            self.get_logger().warn("No drive commands found in CSV.")
            sys.exit(0)

        # Időzítők kezdőpontja
        self._start_ros_time = self.get_clock().now().nanoseconds * 1e-9
        self._start_csv_time = self._rows[0]['time']

        # Publisher-ek
        self._pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self._clock_pub = self.create_publisher(Clock, '/clock', 10)

        # Timer callback a visszajátszáshoz
        self._timer = self.create_timer(0.01, self._timer_callback)

    def _timer_callback(self):
        # Ha elfogytak a sorok, kilépünk
        if self._idx >= len(self._rows):
            self.get_logger().info("Vége a drive_replay-nek.")
            sys.exit(0)

        # ROS-idő és CSV-idő összehasonlítása
        now_ros = self.get_clock().now().nanoseconds * 1e-9 - self._start_ros_time
        next_csv_t = self._rows[self._idx]['time'] - self._start_csv_time
        if now_ros + 1e-6 < next_csv_t:
            return

        row = self._rows[self._idx]
        # /clock publikálása
        clk = Clock()
        t = row['time'] - self._start_csv_time
        clk.clock.sec = int(t)
        clk.clock.nanosec = int((t - clk.clock.sec) * 1e9)
        self._clock_pub.publish(clk)

        # Ackermann üzenet publikálása
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = row['speed']
        msg.drive.steering_angle = row['steering_angle']
        self._pub.publish(msg)

        self._idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = DriveReplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
