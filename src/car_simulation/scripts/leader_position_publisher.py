#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import carla
import time
import math

def euler_to_quaternion(yaw):
    # Mivel csak a yaw szöget használjuk, roll = pitch = 0
    q = PoseStamped().pose.orientation
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    return q

class LeaderPositionPublisher(Node):
    def __init__(self):
        super().__init__('leader_position_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'leader_pose', 10)

        # Carla kapcsolat
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        self.vehicle = None
        self.wait_for_vehicle(timeout=30)

        if self.vehicle is not None:
            self.get_logger().info('Ego vehicle megtalálva, pozíció publikálás indul.')
            self.timer = self.create_timer(0.5, self.publish_position)

    def wait_for_vehicle(self, timeout=30):
        self.get_logger().info('Várakozás az "ego_vehicle" járműre...')
        start_time = time.time()
        while time.time() - start_time < timeout:
            actors = self.world.get_actors().filter('vehicle.*')
            for actor in actors:
                if actor.attributes.get('role_name') == 'ego_vehicle':
                    self.vehicle = actor
                    return
            time.sleep(0.5)
        self.get_logger().error('Nem található "ego_vehicle" jármű a megadott időn belül.')

    def publish_position(self):
        transform = self.vehicle.get_transform()
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = transform.location.x
        pose.pose.position.y = transform.location.y
        pose.pose.position.z = transform.location.z
        pose.pose.orientation = euler_to_quaternion(math.radians(transform.rotation.yaw))

        self.publisher_.publish(pose)
        self.get_logger().info(f"Leader pozíció: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = LeaderPositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
