#!/usr/bin/env python3

import os, csv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomSaver(Node):
    def __init__(self):
        super().__init__('save_odom')
        # csak egyszer deklaráljuk a paramétert
        from rclpy.exceptions import ParameterAlreadyDeclaredException
        try:
            self.declare_parameter('use_sim_time', True)
        except ParameterAlreadyDeclaredException:
            pass

        # Paraméter deklarációk:
        self.declare_parameter('output_dir', '')
        self.declare_parameter('output_filename', 'odom.csv')

        # Paraméterek beolvasása
        out_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        filename = self.get_parameter('output_filename').get_parameter_value().string_value
        # Paraméterek ellenőrzése
        if not out_dir:
            raise RuntimeError("output_dir parameter is required")
        self.filepath = os.path.join(out_dir, filename)
        self.get_logger().info(f"Logging /odom into: {self.filepath}")

        # CSV fájl megnyitása és fejléc írása
        self.csvfile = open(self.filepath, 'w', newline='')
        self._writer = csv.writer(self.csvfile)
        self._writer.writerow(['sec', 'nanosec',
                               'x', 'y', 'z',
                               'qx', 'qy', 'qz', 'qw']),

        # Feliratkozás az /odom topicra
        self.sub = self.create_subscription(
            Odometry, '/odom',
            self._odom_callback,
            10
        )

    def _odom_callback(self, msg: Odometry):
        ts = msg.header.stamp
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._writer.writerow([
            ts.sec, ts.nanosec,
            p.x, p.y, p.z,
            q.x, q.y, q.z, q.w
        ])
        # Írjuk azonnal a lemezes cache-be
        self.csvfile.flush()

    def destroy_node(self):
        # Fájl bezárása node leállításkor
        try:
            self.csvfile.close()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OdomSaver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
