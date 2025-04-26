#!/usr/bin/env python3
"""
ROS2 node for replaying AckermannDriveStamped commands from a CSV file.
Reads a CSV with columns: sec,nanosec,speed,steering_angle (or a combined time column).
Publishes to /drive topic using use_sim_time.
"""
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import pandas as pd

class DriveReplay(Node):
    def __init__(self):
        super().__init__('drive_replay')
        # Parameters
        self.declare_parameter('csv_file', 'filtered_drive_minmax.csv')
        self.declare_parameter('use_sim_time', True)
        csv_path = self.get_parameter('csv_file').get_parameter_value().string_value
        use_sim = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.get_logger().info(f"use_sim_time: {use_sim}, csv: {csv_path}")

        # Publisher
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Load commands
        df = pd.read_csv(csv_path)
        # Compute a unified time column if needed
        if 'sec' in df.columns and 'nanosec' in df.columns:
            df['time'] = df['sec'] + df['nanosec'] * 1e-9
        # Expect columns 'speed' and 'steering_angle'
        self.commands = df[['time', 'speed', 'steering_angle']].to_numpy()

        # State
        self._start_time = None  # ROS time offset
        self._idx = 0

        # Timer for replay loop
        self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds * 1e-9  # seconds
        # Initialize start_time on first callback where now >= first command time
        if self._start_time is None:
            t_cmd0 = float(self.commands[0,0])
            if now >= t_cmd0:
                self._start_time = now - t_cmd0
            else:
                return
        # Publish all commands whose scheduled time has arrived
        while self._idx < len(self.commands):
            t_cmd, speed, steering = map(float, self.commands[self._idx])
            if t_cmd + self._start_time <= now:
                msg = AckermannDriveStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.drive.speed = speed
                msg.drive.steering_angle = steering
                self.pub.publish(msg)
                self._idx += 1
            else:
                break

        # Optionally: shutdown once done
        if self._idx >= len(self.commands):
            self.get_logger().info('All commands published, shutting down.')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DriveReplay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
