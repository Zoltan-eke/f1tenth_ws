#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
import carla
import math


class LeaderPositionPublisher(Node):
    def __init__(self):
        super().__init__('leader_position_publisher')

        self.pub = self.create_publisher(PoseStamped, 'initial_pose', 10)
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        # Timer: folyamatos publikálás 10 Hz-en
        self.timer = self.create_timer(0.1, self.publish_pose)

    def publish_pose(self):
                
        actors = self.world.get_actors().filter('vehicle.*')
        leader = next((a for a in actors if a.attributes.get('role_name')=='leader'), None)
        if leader is None:
            return
        
        tf = leader.get_transform()
        ps = PoseStamped()
        ps.header.stamp   = self.get_clock().now().to_msg()
        ps.header.frame_id= 'leader'
        ps.pose.position.x = tf.location.x
        ps.pose.position.y = tf.location.y
        ps.pose.position.z = tf.location.z

        # CARLA-ban rotation.yaw fokban, felfelé pozitív; konvertáljuk radiánba
        yaw_rad = math.radians(tf.rotation.yaw)
        # csak a z-tengely körüli elforgatás van, roll/pitch = 0
        q = Quaternion()
        q.w = math.cos(yaw_rad * 0.5)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw_rad * 0.5)
        ps.pose.orientation = q

        self.pub.publish(ps)
        self.get_logger().info(f'Leader pose published: x={ps.pose.position.x:.2f}, y={ps.pose.position.y:.2f}, yaw={tf.rotation.yaw:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = LeaderPositionPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
