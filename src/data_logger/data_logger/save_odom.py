#!/usr/bin/env python3

import os, csv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomSaver(Node):
    def __init__(self):
        super().__init__('save_odom')
        # Paraméter deklarációk
        self.declare_parameter('output_file', 'traj.csv')
        output_file = self.get_parameter('output_file').value

        # Kimeneti könyvtár létrehozása
        out_dir = os.path.dirname(output_file) or '.'
        os.makedirs(out_dir, exist_ok=True)

        # CSV fájl megnyitása és fejléc írása
        self._f = open(output_file, 'w', newline='')
        self._writer = csv.writer(self._f)
        self._writer.writerow(['sec', 'nanosec',
                               'x', 'y', 'z',
                               'qx', 'qy', 'qz', 'qw'])
        self.get_logger().info(f'Logging /odom into: {output_file}')

        # Feliratkozás az /odom topicra
        self.create_subscription(
            Odometry,
            '/odom',
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
        self._f.flush()

    def destroy_node(self):
        # Fájl bezárása node leállításkor
        try:
            self._f.close()
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
