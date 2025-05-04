#!/usr/bin/env python3

import os, csv, rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class SaveAckermannCmd(Node):
    def __init__(self):
        super().__init__('save_ackermann_cmd')

        # Enable simulated time
        from rclpy.exceptions import ParameterAlreadyDeclaredException
        try:
            self.declare_parameter('use_sim_time', True)
        except ParameterAlreadyDeclaredException:
            pass

        # Which topic to log (bag /drive or sim /sim_drive)
        self.declare_parameter('ackermann_cmd_topic', '/drive')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('output_filename', 'drive.csv')

        topic    = self.get_parameter('ackermann_cmd_topic')\
                           .get_parameter_value().string_value
        out_dir  = self.get_parameter('output_dir')\
                           .get_parameter_value().string_value
        filename = self.get_parameter('output_filename')\
                           .get_parameter_value().string_value

        if not out_dir:
            raise RuntimeError("output_dir parameter is required")
        os.makedirs(out_dir, exist_ok=True)

        self.filepath = os.path.join(out_dir, filename)
        self.get_logger().info(f"Logging '{topic}' into: {self.filepath}")

        # Open CSV and write header
        self.csvfile = open(self.filepath, 'w', newline='')
        self._writer = csv.writer(self.csvfile)
        self._writer.writerow(['sec', 'nanosec', 'speed', 'steering_angle'])

        # Subscribe to the chosen topic
        self._sub = self.create_subscription(
            AckermannDriveStamped,
            topic,
            self._cb,
            10
        )

    def _cb(self, msg: AckermannDriveStamped):
        now   = self.get_clock().now().to_msg()
        sec   = now.sec
        nsec  = now.nanosec
        speed = msg.drive.speed
        steer = msg.drive.steering_angle

        self._writer.writerow([sec, nsec, speed, steer])
        self.csvfile.flush()

    def destroy_node(self):
        try:
            self.csvfile.close()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
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