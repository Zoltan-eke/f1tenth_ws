#!/usr/bin/env python3
import rclpy, os, csv, math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class SimulationLogger(Node):
    def __init__(self):
        super().__init__('simulation_logger')
        out = self.declare_parameter('output_dir','/tmp/log').value
        os.makedirs(out, exist_ok=True)

        # leader
        self.leader_f = open(os.path.join(out,'leader.csv'),'w', newline='')
        self.lw       = csv.writer(self.leader_f)
        self.lw.writerow(['t','x','y','z'])

        # control
        self.cmd_f = open(os.path.join(out,'control.csv'),'w', newline='')
        self.cw    = csv.writer(self.cmd_f)
        self.cw.writerow(['t','steer','speed'])

        # --- real.csv (leader_pose-ból) ---
        self.real_f = open(os.path.join(out,'real.csv'),'w', newline='')
        self.rw     = csv.writer(self.real_f)
        self.rw.writerow(['t','x','y','z','yaw'])

        # --- model.csv ---
        self.model_f = open(os.path.join(out,'model.csv'),'w', newline='')
        self.mw      = csv.writer(self.model_f)
        self.mw.writerow(['t','x','y','z','yaw'])

        # Feliratkozások
        # Leader pozíció (PoseStamped)
        self.create_subscription(
            PoseStamped,
            'initial_pose',
            self.cb_leader,
            10
        )
        # Control parancsok
        self.create_subscription(
            AckermannDriveStamped,
            '/carla/ego_vehicle/ackermann_cmd',
            self.cb_ctrl,
            10
        )
        # Real odom helyett újra leader_pose-ból, hogy legyen real.csv
        self.create_subscription(
            PoseStamped,
            'initial_pose',
            self.cb_real,
            10
        )
        # Modell-kimenet
        self.create_subscription(
            Odometry,
            'follower_odom',
            self.cb_model,
            10
        )

        self.get_logger().info(f'SimulationLogger initialized, logging to {out}')

    def cb_leader(self, msg: PoseStamped):
        t = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
        p = msg.pose.position
        self.lw.writerow([t, p.x, p.y, p.z])
        self.leader_f.flush()

    def cb_ctrl(self, msg: AckermannDriveStamped):
        t = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
        self.cw.writerow([t, msg.drive.steering_angle, msg.drive.speed])
        self.cmd_f.flush()

    def cb_real(self, msg: PoseStamped):
        # itt a leader_pose-ból logoljuk a real odomot
        t = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
        p = msg.pose.position
        # a PoseStampedből most már van orientation
        q = msg.pose.orientation
        # yaw = 2 * atan2(z, w)
        yaw = 2 * math.atan2(q.z, q.w)
        self.rw.writerow([t, p.x, p.y, p.z, yaw])
        self.real_f.flush()

    def cb_model(self, msg: Odometry):
        t = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = 2 * math.atan2(q.z, q.w)
        self.mw.writerow([t, p.x, p.y, p.z, yaw])
        self.model_f.flush()

    def destroy_node(self):
        # fájlok lezárása indításkor
        self.leader_f.close()
        self.cmd_f.close()
        self.real_f.close()
        self.model_f.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SimulationLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()