#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction, EmitEvent
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.events import Shutdown

def generate_launch_description():
#    pkg_log = get_package_share_directory('data_logger')

    # --- Paraméterek ---
    bag_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=os.path.join(
            os.getenv('HOME'),
            'f1tenth_ws',
            'bags',
            'rosbag2_2025_03_18-15_37_52_0'
        ),
        description='Path to the ros2 bag directory'
    )

    offset_arg = DeclareLaunchArgument(
        'start_offset',
        default_value='0.0',
        description='Seconds to skip at beginning of bag'
    )

     # --- CSV kimeneti mappa ---
    output_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=os.path.join(os.getenv('HOME'),
                                   'f1tenth_ws', 'src', 'data_logger',
                                   'logged_file', 'logged_from_rosbag'),
        description='Directory where to write CSV files'
    )

    # --- output file paths ---
    odom_csv = PathJoinSubstitution([
        LaunchConfiguration('output_dir'),
        'odom.csv'
    ])
    imu_csv = PathJoinSubstitution([
        LaunchConfiguration('output_dir'),
        'imu.csv'
    ])
    js_csv = PathJoinSubstitution([
        LaunchConfiguration('output_dir'),
        'joint_states.csv'
    ])

    ackermann_csv = PathJoinSubstitution([
        LaunchConfiguration('output_dir'),
        'ackermann_cmd.csv'
    ])

    # --- Logger node-ok ---
    odom_logger = Node(
        package='data_logger',
        executable='save_odom',
        name='save_odom',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_topic': '/odom',
            'output_file': odom_csv
        }]
    )

    imu_logger = Node(
        package='data_logger',
        executable='save_imu',
        name='save_imu',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'imu_topic': '/sensors/imu',
            'output_file': imu_csv
        }]
    )

    joint_states_logger = Node(
        package='data_logger',
        executable='save_joint_states',
        name='save_joint_states',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'joint_states_topic': '/joint_states',
            'output_file': js_csv
        }]
    )

    ackermann_logger = Node(
        package='data_logger',
        executable='save_ackermann_cmd',
        name='save_ackermann_cmd',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'ackermann_topic': '/ackermann_cmd',
            'output_file': ackermann_csv
        }]
    )

    # --- ros2 bag play (Humble: use -p for start offset) ---
    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', LaunchConfiguration('bag_path'),
            '--clock', LaunchConfiguration('start_offset')
        ],
        output='screen'
    )

    # --- (Opcionális) automatikus leállítás pl. 90 másodperc után --- 
    shutdown_after = TimerAction(
         period=87.5,
         actions=[EmitEvent(event=Shutdown())]
     )

    return LaunchDescription([
        bag_arg,
        offset_arg,
        output_arg,
        # bufferelt loggolást kikapcsoljuk sim_time esetén
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        # Először indítsuk el a logger‑node‑okat,
        # hogy a /clock topic – amit a bag_play biztosít – fogadókész legyen.
        TimerAction(period=0.5, actions=[odom_logger, imu_logger, joint_states_logger, ackermann_logger]),
        # 1 mp késleltetéssel indítjuk el a bag_play‑t
        TimerAction(period=1.0, actions=[bag_play]),
        # ha akarsz automatikus leállítást, vedd vissza a kommentet:
        shutdown_after,
    ])
