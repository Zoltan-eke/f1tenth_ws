#!/usr/bin/env python3
"""
Launch file for F1TENTH car_simulation with xacro argument support.
"""
import os
import glob
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, RegisterEventHandler, Shutdown
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

WORKSPACE_ROOT = os.path.expanduser('~/f1tenth_ws')
BAGS_DIR       = os.path.join(WORKSPACE_ROOT,'bags')

def find_latest_bag_dir():
    subs = [d for d in glob.glob(f"{BAGS_DIR}/*") if os.path.isdir(d)]
    return max(subs, key=os.path.getmtime)

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
                    'xacro ', LaunchConfiguration('model')
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

    filtered_dir = os.path.expanduser(
        '~/f1tenth_ws/src/data_logger/logged_file/filtered'
    )
    os.makedirs(filtered_dir, exist_ok=True)

    # 4) Save odom to filtered folder after the bag is played
    save_odom = Node(
        package='data_logger',
        executable='save_odom',
        name='save_odom',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'output_dir': filtered_dir}
        ]
    )

    # 5) Save tf to filtered folder after the bag is played
    save_tf = Node(
        package='data_logger',
        executable='save_tf',
        name='save_tf',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'output_dir': filtered_dir}
        ]
    )

    # 6) Save ackermann_cmd to filtered folder after the bag is played
    save_ackermann_cmd = Node(
        package='data_logger',
        executable='save_ackermann_cmd',
        name='save_ackermann_cmd',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'ackermann_cmd_topic': '/drive'},
            {'output_dir': filtered_dir},
        ]
    )
    
    # 7) Save IMU data to filtered folder after the bag is played
    # Note: The IMU topic is hardcoded to '/sensors/imu/raw' in the save_imu node
    save_imu = Node(
        package='data_logger',
        executable='save_imu',
        name='save_imu',
        parameters=[
        {'use_sim_time': True},
        {'imu_topic': '/sensors/imu/raw'},
        {'output_dir': filtered_dir},
        {'output_filename': 'imu.csv'}
        ]
    )

    shutdown_handler = RegisterEventHandler(
        OnProcessExit(target_action=rosbag_play, on_exit=[Shutdown()])
    )

    return LaunchDescription([
        declare_model_arg,
        TimerAction(period=0.5, actions=[robot_state_publisher]),
        TimerAction(period=0.5, actions=[rosbag_play]),
        TimerAction(period=2.0, actions=[save_odom, save_tf, save_ackermann_cmd, save_imu]),
        shutdown_handler
    ])
