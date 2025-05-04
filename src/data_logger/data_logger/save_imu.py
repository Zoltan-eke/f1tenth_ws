#!/usr/bin/env python3

import os
import csv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuSaver(Node):
    def __init__(self):
        super().__init__('save_imu')

        # sim-time paraméter
        from rclpy.exceptions import ParameterAlreadyDeclaredException
        try:
            self.declare_parameter('use_sim_time', True)
        except ParameterAlreadyDeclaredException:
            pass

        # topic, output dir és filename
        self.declare_parameter('imu_topic', '/sensors/imu/raw')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('output_filename', 'imu.csv')

        topic     = self.get_parameter('imu_topic')\
                            .get_parameter_value().string_value
        out_dir   = self.get_parameter('output_dir')\
                            .get_parameter_value().string_value
        filename  = self.get_parameter('output_filename')\
                            .get_parameter_value().string_value

        if not out_dir:
            raise RuntimeError("‘output_dir’ parameter is required")
        os.makedirs(out_dir, exist_ok=True)

        self._filepath = os.path.join(out_dir, filename)
        self.get_logger().info(f"Logging IMU from '{topic}' into: {self._filepath}")

        # CSV file init
        self._f = open(self._filepath, 'w', newline='')
        self._writer = csv.writer(self._f)
        self._writer.writerow([
            'sec','nanosec',
            'ax','ay','az',
            'gx','gy','gz'
        ])

        # Sub to IMU topic
        self._sub = self.create_subscription(
            Imu, topic, self._cb, 10
        )

    def _cb(self, msg: Imu):
        ts = msg.header.stamp
        a  = msg.linear_acceleration
        g  = msg.angular_velocity
        self._writer.writerow([
            ts.sec, ts.nanosec,
            a.x, a.y, a.z,
            g.x, g.y, g.z
        ])
        self._f.flush()

    def destroy_node(self):
        try:
            self._f.close()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImuSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
