#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import csv
import os

class AckermannCmdLogger(Node):
    def __init__(self):
        super().__init__('save_ackermann_cmd')

        # Paraméterek betöltése
        self.declare_parameter('ackermann_cmd_topic', '/ackermann_cmd')
        self.declare_parameter('output_file', 'ackermann_cmd.csv')
        topic = self.get_parameter('ackermann_cmd_topic').get_parameter_value().string_value
        output_file = self.get_parameter('output_file').get_parameter_value().string_value

        # Kimeneti mappa létrehozása
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        self._f = open(output_file, 'w', newline='')
        self._writer = csv.writer(self._f)
        self._writer.writerow(['sec', 'nanosec', 'speed', 'steering_angle'])

        self.get_logger().info(f"Logging {topic} into: {output_file}")

        # Feliratkozás a kiválasztott topikra
        self.create_subscription(AckermannDriveStamped, topic, self._callback, 10)

    def _callback(self, msg: AckermannDriveStamped):
        ts = msg.header.stamp
        speed = msg.drive.speed
        steering_angle = msg.drive.steering_angle
        self._writer.writerow([ts.sec, ts.nanosec, speed, steering_angle])
        self._f.flush()

    def destroy_node(self):
        self._f.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AckermannCmdLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
