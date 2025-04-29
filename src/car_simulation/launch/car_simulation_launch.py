#!/usr/bin/env python3
import os, glob
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, RegisterEventHandler, Shutdown
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

WORKSPACE_ROOT = os.path.expanduser('~/f1tenth_ws')
BAGS_DIR       = os.path.join(WORKSPACE_ROOT, 'src', 'car_simulation', 'bags')
SIM_DIR        = os.path.join(WORKSPACE_ROOT, 'src', 'data_logger', 'logged_file', 'simulated')

def find_latest_bag_dir():
    subs = [d for d in glob.glob(f"{BAGS_DIR}/*") if os.path.isdir(d)]
    return max(subs, key=os.path.getmtime)

def generate_launch_description():
    bag = find_latest_bag_dir()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', PathJoinSubstitution([
                FindPackageShare('car_description'),
                'urdf', 'base_link.xacro'
            ])]),
            'use_sim_time': True
        }],
        output='screen'
    )

    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag, '--clock', '--rate', '1.0'],
        output='screen'
    )

#    drive_replay = Node(
#        package='car_simulation',
#        executable='drive_replay.py',
#       output='screen',
#        parameters=[{'use_sim_time': True}],
#        remappings=[('/ackermann_cmd', '/ackermann_replay_cmd')],
#    )

    ackermann_to_odom = Node(
        package='car_simulation', executable='ackermann_to_odom.py',
        parameters=[{'wheel_base': 0.320},
#                   {'use_sim_time': True}
                ],
        output='screen',
        remappings=[
#                    ('/ackermann_cmd', '/ackermann_replay_cmd'),
                    ('/odom', '/sim_odom')],
    )

    os.makedirs(SIM_DIR, exist_ok=True)
    save_real = Node(
        package='data_logger',
        executable='save_odom',
        parameters=[{'use_sim_time': True},
                    {'output_dir': SIM_DIR},
                    {'output_filename': 'real_odom.csv'}]
    )
    save_sim = Node(
        package='data_logger',
        executable='save_odom',
        parameters=[{'use_sim_time': True},
                    {'output_dir': SIM_DIR},
                    {'output_filename': 'sim_odom.csv'}],
        remappings=[('/odom', '/sim_odom')]
    )

    shutdown_handler = RegisterEventHandler(
        OnProcessExit(target_action=bag_play, on_exit=[Shutdown()])
    )

    return LaunchDescription([
        TimerAction(period=0.5, actions=[rsp]),
        TimerAction(period=1.0, actions=[bag_play]),
        TimerAction(period=1.5, actions=[ackermann_to_odom]),
        TimerAction(period=1.6, actions=[save_real, save_sim]),
        shutdown_handler
    ])
