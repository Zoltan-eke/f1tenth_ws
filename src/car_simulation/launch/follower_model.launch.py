from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Paraméter fájl teljes útvonala
    param_file_path = os.path.join(
        os.getenv('HOME'),
        'f1tenth_ws',
        'src',
        'car_description',
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='car_simulation',
            executable='vehicle_model_node.py',
            name='follower_vehicle_model',
            output='screen',
            parameters=[param_file_path]
        ),
        Node(
            package='car_simulation',
            executable='leader_position_publisher.py',
            name='leader_position_publisher',
            output='screen',
        )
    ])
