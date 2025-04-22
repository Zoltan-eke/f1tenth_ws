#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import csv

class SaveAckermannCmd(Node):
    def __init__(self):
        # Node name should match the launch-executable
        super().__init__('save_drive_cmd')

        # Parameters for topic and output file
        ack_topic_param = self.declare_parameter('ackermann_cmd_topic', '/drive')
        output_file_param = self.declare_parameter('output_file', 'drive.csv')

        ack_topic = ack_topic_param.get_parameter_value().string_value
        output_file = output_file_param.get_parameter_value().string_value

        # Subscribe to AckermannDriveStamped topic
        self.sub = self.create_subscription(
            AckermannDriveStamped,
            ack_topic,
            self.cb,
            10
        )

        # Open CSV file and write header
        self.csvfile = open(output_file, 'w', newline='')
        self.writer = csv.writer(self.csvfile)
        self.writer.writerow(['sec', 'nanosec', 'speed', 'steering_angle'])

    def cb(self, msg: AckermannDriveStamped):
        # Always use the ROS clock for timestamp, ignoring msg.header
        now = self.get_clock().now().to_msg()
        sec = now.sec
        nanosec = now.nanosec

        # Write timestamp and command values to CSV
        self.writer.writerow([
            sec,
            nanosec,
            msg.drive.speed,
            msg.drive.steering_angle
        ])

    def destroy_node(self):
        # Ensure the CSV file is closed
        try:
            self.csvfile.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = SaveAckermannCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
