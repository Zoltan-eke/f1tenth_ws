# scripts/save_joint_states.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import csv, os

class JointsSaver(Node):
    def __init__(self):
        super().__init__('save_joints')
        self.declare_parameter('output_file', 'joints.csv')
        out = self.get_parameter('output_file').value
        os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
        self._f = open(out, 'w', newline='')
        w = csv.writer(self._f)
        # fejléc a joint nevek szerint
        w.writerow(['sec','nanosec'] + ['pos_'+j for j in ['lf','rf','lr','rr']])
        self.get_logger().info(f'Logging /joint_states into: {out}')
        self.create_subscription(JointState, '/joint_states', self.cb, 10)

    def cb(self, msg):
        ts = msg.header.stamp
        # példa: az első 4 joint pozíciója
        poses = msg.position[:4]
        csv.writer(self._f).writerow([ts.sec, ts.nanosec, *poses])
        self._f.flush()

    def destroy_node(self):
        self._f.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(); node=JointsSaver()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
