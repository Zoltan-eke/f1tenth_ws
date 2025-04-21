# scripts/save_imu.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import csv, os

class ImuSaver(Node):
    def __init__(self):
        super().__init__('save_imu')
        self.declare_parameter('output_file', 'imu.csv')
        out = self.get_parameter('output_file').value
        os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
        self._f = open(out, 'w', newline='')
        w = csv.writer(self._f)
        w.writerow(['sec','nanosec','ax','ay','az','gx','gy','gz'])
        self.get_logger().info(f'Logging /imu into: {out}')
        self.create_subscription(Imu, '/sensors/imu/raw', self.cb, 10)

    def cb(self, msg):
        ts = msg.header.stamp
        a = msg.linear_acceleration
        g = msg.angular_velocity
        csv.writer(self._f).writerow([
            ts.sec, ts.nanosec,
            a.x, a.y, a.z,
            g.x, g.y, g.z
        ])
        self._f.flush()

    def destroy_node(self):
        self._f.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args); node=ImuSaver(); 
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
