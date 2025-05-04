from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='car_simulation',
            executable='closed_loop_controller.py',
            name='closed_loop_controller',
            output='screen',
            parameters=[{
                # Itt paraméterezheted a célsebességet
                'target_speed': 1.0
            }]
        ),
    ])
