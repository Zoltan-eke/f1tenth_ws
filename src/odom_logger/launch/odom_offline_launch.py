# odom_logger/launch/odom_offline_launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --------------------------------------------------
    # 1. Argumentumok
    bag_arg = DeclareLaunchArgument(
        'bag_path',
        description='Útvonal a rosbag mappájához'
    )
    offset_arg = DeclareLaunchArgument(
        'start_offset',
        default_value='0.0',
        description='Indítási késleltetés a bag elején (s)'
    )
    out_arg = DeclareLaunchArgument(
        'output_file',
        default_value=os.path.join(
            get_package_share_directory('odom_logger'),
            'logged_file','real_traj.csv'
        ),
        description='CSV kimeneti fájl'
    )

    bag_path     = LaunchConfiguration('bag_path')
    start_offset = LaunchConfiguration('start_offset')
    output_file  = LaunchConfiguration('output_file')

    # --------------------------------------------------
    # 2. robot_state_publisher – Xacro feldolgozás
    car_pkg     = 'car_description'
    xacro_file  = os.path.join(
        get_package_share_directory(car_pkg),
        'urdf','base_link.xacro'
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            # Ezt NE egyszerű stringként add meg, hanem Command-cal!
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file]),
                value_type=str
            ),
            'use_sim_time': True
        }]
    )

    # --------------------------------------------------
    # 3. odom_logger node
    odom_logger = Node(
        package='odom_logger',
        executable='save_odom',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'output_file': output_file}
        ]
    )

    # --------------------------------------------------
    # 4. ros2 bag play
    bag_play = ExecuteProcess(
        cmd=[
            'ros2','bag','play', bag_path,
            '--clock',
            '--start-offset', start_offset
        ],
        output='screen'
    )

    # --------------------------------------------------
    # Indítási sorrend:  
    #   1. robot_state_publisher  
    #   2. 1 s múlva odom_logger  
    #   3. 2 s múlva bag play
    return LaunchDescription([
        bag_arg, offset_arg, out_arg,
        robot_state_publisher,
        TimerAction(period=1.0, actions=[odom_logger]),
        TimerAction(period=2.0, actions=[bag_play]),
    ])
