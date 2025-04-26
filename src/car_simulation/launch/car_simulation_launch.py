#!/usr/bin/env python3
"""
Launch file for F1TENTH car_simulation:
 - lejátssza a legújabb rosbag-et
 - robot_state_publisher betölti az URDF-et xacro-val
 - a data_logger két külön mentést végez a simulated mappába:
   * real_odom (eredeti bag /odom)
   * sim_odom (drive_replay által generált /odom)
"""
import os
import glob
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, RegisterEventHandler, Shutdown
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

# Mappák és fájlok útvonalai
WORKSPACE_ROOT = os.path.expanduser('~/f1tenth_ws')
# Bag-ek helye
BAGS_DIR = os.path.join(WORKSPACE_ROOT, 'src', 'car_simulation', 'bags')
# Simultált mentések helye
SIM_DIR = os.path.join(WORKSPACE_ROOT, 'src', 'data_logger', 'logged_file', 'simulated')

# Segédfüggvény: legutolsó bag-könyvtár
def find_latest_bag_dir():
    subs = [d for d in glob.glob(os.path.join(BAGS_DIR, '*')) if os.path.isdir(d)]
    if not subs:
        raise FileNotFoundError(f"No bag directories in {BAGS_DIR}")
    return max(subs, key=os.path.getmtime)


def generate_launch_description():
    # 1) XACRO betöltése a car_description csomagból
    xacro_path = PathJoinSubstitution([
        FindPackageShare('car_description'),
        'urdf', 'base_link.xacro'
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
            'robot_description':Command(['xacro ', xacro_path]),            
            'use_sim_time': True}
        ]
    )

    # 2) ROS2 bag lejátszása a legújabb könyvtárból
    bag_dir = find_latest_bag_dir()
    rosbag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_dir,
            '--clock', '--rate', '1.0'
        ],
        output='screen'
    )

    # 3) Drive replay node /ackermann_cmd -> /sim_odom
    drive_replay = Node(
        package='car_simulation',
        executable='drive_replay.py',
        name='drive_replay',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[('odom', 'sim_odom')]
    )

    # 4) Mentések: real_odom és sim_odom
    os.makedirs(SIM_DIR, exist_ok=True)
    # 4a) Valódi odom mentése real_odom.csv
    save_real = ExecuteProcess(
    cmd=[
        'ros2','run','data_logger','save_odom',
        '--ros-args',
        '-p','use_sim_time:=true',
        '-p',f'output_dir:={SIM_DIR}',
        '-p','output_filename:=real_odom.csv'
    ],
    output='screen'
    )
    # 4b) Szimulált odom mentése sim_odom.csv
    save_sim = ExecuteProcess(
    cmd=[
        'ros2','run','data_logger','save_odom',
        '--ros-args',
        '-p','use_sim_time:=true',
        '-p',f'output_dir:={SIM_DIR}',
        '-p','output_filename:=sim_odom.csv'
    ],
    output='screen'
    )

    # ha a bag_play véget ér, shutdown-oljuk az egész launch-ot
    shutdown_on_bag_end = RegisterEventHandler(
        OnProcessExit(
            target_action=rosbag_play,
            on_exit=[Shutdown()]
        )
    )



    return LaunchDescription([
        TimerAction(period=0.5, actions=[robot_state_publisher]),
        TimerAction(period=1.0, actions=[rosbag_play]),
        TimerAction(period=1.1, actions=[drive_replay]),
        TimerAction(period=1.2, actions=[save_real]),
        TimerAction(period=1.2, actions=[save_sim]),
        shutdown_on_bag_end
    ])
