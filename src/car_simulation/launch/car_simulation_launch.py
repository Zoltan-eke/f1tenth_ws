#!/usr/bin/env python3
import os, glob
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, RegisterEventHandler, Shutdown
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

WORKSPACE_ROOT = os.path.expanduser('~/f1tenth_ws')
BAGS_DIR       = os.path.join(WORKSPACE_ROOT, 'bags')
SIM_DIR        = os.path.join(WORKSPACE_ROOT, 'src', 'data_logger', 'logged_file', 'simulated')

def find_latest_bag_dir():
    subs = [d for d in glob.glob(f"{BAGS_DIR}/*") if os.path.isdir(d)]
    return max(subs, key=os.path.getmtime)

def generate_launch_description():
    bag = find_latest_bag_dir()

    pkg_share = FindPackageShare('car_description').find('car_description')
    params_file = PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])
    xacro_file  = os.path.join(pkg_share, 'urdf', 'base_link.xacro')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': True},
            {'robot_description': Command([
            'xacro ', xacro_file])},   
        ],
    )

    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag, '--clock', '--rate', '1.0'],
        output='screen'
    )

    sim_vehicle = Node(
        package='car_simulation', executable='ackermann_to_odom.py',
        name='sim_vehicle',
        parameters=[{'wheel_base': 0.320},
                    {'use_sim_time': True},
                    {'drive_topic': '/drive'},
                    ],
        output='screen',
        remappings=[
                    ('/odom', '/sim_odom'),
                    ('/tf', '/sim_tf'),      
                ],
    )

    os.makedirs(SIM_DIR, exist_ok=True)

    save_sim_odom = Node(
        package='data_logger',
        executable='save_odom',
        parameters=[{'use_sim_time': True},
                    {'output_dir': SIM_DIR},
                    {'output_filename': 'sim_odom.csv'}],
        remappings=[('/odom', '/sim_odom')]
    )

    save_sim_tf = Node(
        package='data_logger',
        executable='save_tf',
        parameters=[{'use_sim_time': True},
                    {'output_dir': SIM_DIR},
                    {'output_filename': 'sim_tf.csv'}],
        remappings=[('/tf', '/sim_tf')]
    )

    save_sim_ackermann_cmd = Node(
        package='data_logger',
        executable='save_ackermann_cmd',
        parameters=[{'use_sim_time': True},
                    {'output_dir': SIM_DIR},
                    {'output_filename': 'sim_drive.csv'}],
            )

    shutdown_handler = RegisterEventHandler(
        OnProcessExit(target_action=bag_play, on_exit=[Shutdown()])
    )

    return LaunchDescription([
        TimerAction(period=0.5, actions=[robot_state_publisher]),
        TimerAction(period=1.0, actions=[bag_play]),
        TimerAction(period=1.5, actions=[sim_vehicle]),
        TimerAction(period=1.6, actions=[save_sim_tf, save_sim_odom, save_sim_ackermann_cmd]),
        shutdown_handler
    ])
