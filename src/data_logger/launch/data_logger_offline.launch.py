#!/usr/bin/env python3
"""
Launch file for F1TENTH car_simulation with xacro argument support.
"""
import os
import glob
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def find_latest_bag_dir():
    base = os.path.expanduser('~/f1tenth_ws/src/car_simulation/bags')
    subdirs = [d for d in glob.glob(os.path.join(base, '*')) if os.path.isdir(d)]
    if not subdirs:
        raise FileNotFoundError(f"No bag directories found in {base}")
    return sorted(subdirs, key=os.path.getmtime, reverse=True)[0]


def generate_launch_description():
    # 1) Declare launch argument for xacro model path
    default_model = os.path.expanduser('~/f1tenth_ws/src/car_description/urdf/base_link.xacro')
    declare_model_arg = DeclareLaunchArgument(
        'model', default_value=default_model,
        description='Path to robot XACRO file'
    )

    # 2) robot_state_publisher using the launch argument
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': Command([
                    'xacro', LaunchConfiguration('model')
                ])
            },
            {'use_sim_time': True}
        ]
    )

    # 3) Play the latest filtered bag directory
    bag_dir = find_latest_bag_dir()
    rosbag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_dir,
            '--clock', '--rate', '1.0'
        ],
        output='screen'
    )

    # 4) Forward /drive to /ackermann_cmd
    drive_replay = Node(
        package='car_simulation',
        executable='drive_replay',
        name='drive_replay',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 5) Save simulated odom to simulated folder
    simulated_dir = os.path.expanduser(
        '~/f1tenth_ws/src/data_logger/logged_file/simulated'
    )
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
        declare_model_arg,
        TimerAction(period=0.5, actions=[robot_state_publisher]),
        TimerAction(period=1.0, actions=[rosbag_play]),
        TimerAction(period=1.2, actions=[drive_replay]),
        TimerAction(period=1.5, actions=[save_odom]),
    ])
