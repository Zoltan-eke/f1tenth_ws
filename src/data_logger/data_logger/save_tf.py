#!/usr/bin/env python3
import os, csv
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class TFSaver(Node):
    def __init__(self):
        super().__init__('save_tf')
        # use_sim_time parameter
        from rclpy.exceptions import ParameterAlreadyDeclaredException
        try:
            self.declare_parameter('use_sim_time', True)
        except ParameterAlreadyDeclaredException:
            pass

        # Paraméterek
        self.declare_parameter('output_dir', '')
        self.declare_parameter('output_filename', 'tf.csv')

        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        filename = self.get_parameter('output_filename').get_parameter_value().string_value

        if not output_dir:
            raise RuntimeError("'output_dir' parameter is required")
        os.makedirs(output_dir, exist_ok=True)
        
        self.filepath = os.path.join(output_dir, filename)
        self.get_logger().info(f"Logging /tf into: {self.filepath}")

        # open CSV file and write header
        self.csvfile = open(self.filepath, 'w', newline='')
        self._writer = csv.writer(self.csvfile)
        self._writer.writerow([
            'sec', 'nanosec',
            'frame_id', 'child_frame_id',
            'tx', 'ty', 'tz',
            'qx', 'qy', 'qz', 'qw'
        ])

        # subscribe to /tf topic
        self.create_subscription(TFMessage, '/tf', self._tf_callback, 10)

    def _tf_callback(self, msg: TFMessage):
        now = self.get_clock().now().to_msg()
        for t in msg.transforms:
            self._writer.writerow([
                now.sec, now.nanosec,
                t.header.frame_id, t.child_frame_id,
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w
            ])
        self.csvfile.flush()

    def destroy_node(self):
        self.csvfile.close()
        super().destroy_node()

def main():
     rclpy.init()
     node = TFSaver()
     try:
         rclpy.spin(node)
     except KeyboardInterrupt:
         pass
     finally:
         node.destroy_node()
         rclpy.shutdown()

if __name__ == '__main__':
    main()