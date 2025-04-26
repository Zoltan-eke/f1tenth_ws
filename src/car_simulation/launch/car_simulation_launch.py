#!/usr/bin/env python3
import os
import glob
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command


def generate_launch_description():
    # 1) URDF model betöltése xacro-ból a telepített 'car_description' csomagból
    xacro_path = PathJoinSubstitution([
        FindPackageShare('car_description'),
        'urdf',
        'base_link.xacro'
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            # space after 'xacro' ensures correct command
            {'robot_description': Command(['xacro ', xacro_path])},
            {'use_sim_time': True}
        ]
    )

    # 2) Dinamikusan keresett bag útvonala: bármely almappa a bags könyvtárból
    bags_base = os.path.expanduser('~/f1tenth_ws/src/car_simulation/bags')
    matches = glob.glob(os.path.join(bags_base, '*'))  # minden almappa
    if not matches:
        raise FileNotFoundError(f"No bag found in {bags_base}")
    bag_play_path = matches[0]  # az első találat
    rosbag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            bag_play_path,
            '--clock',
            '--rate', '1.0'
        ],
        output='screen'
    )

    # 3) Szimulált odom mentése a data_logger csomag 'simulated' mappájába
    simulated_dir = os.path.expanduser('~/f1tenth_ws/src/data_logger/logged_file/simulated')
    os.makedirs(simulated_dir, exist_ok=True)
    save_odom = Node(
        package='data_logger',
        executable='save_odom',
        name='save_odom',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'output_dir': simulated_dir}
        ]
    )

    return LaunchDescription([
        TimerAction(period=0.5, actions=[robot_state_publisher]),
        TimerAction(period=1.0, actions=[rosbag_play]),
        TimerAction(period=1.0, actions=[save_odom]),
    ])
