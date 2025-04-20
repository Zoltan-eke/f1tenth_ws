import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 1) paraméterek
    pkg_car = get_package_share_directory('car_description')
    pkg_odom = get_package_share_directory('odom_logger')

    bag_arg = DeclareLaunchArgument(
        'bag_path',
        description='Az offline rosbag elérési útvonala',
        default_value=os.path.join(pkg_odom, 'bags', 'your.bag')
    )
    skip_arg = DeclareLaunchArgument(
        'start_offset',
        default_value='0.0',
        description='Hány mp-et ugrunk az elejéből'
    )

    # 2) robot_state_publisher (xacro feldolgozás)
    xacro_file = os.path.join(pkg_car, 'urdf', 'base_link.xacro')
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': True
        }]
    )

    # 3) include save_odom.launch.py
    save_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_odom, 'launch', 'save_odom.launch.py')
        ]),
        launch_arguments={
            'odom_topic': '/odom',
            'use_sim_time': 'true',
            # default CSV-t így írja felül:
            'output_file': os.path.join(pkg_odom, 'logged_file', 'traj_offline.csv')
        }.items()
    )

    # 4) ros2 bag play
    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', LaunchConfiguration('bag_path'),
            '--clock',
            '--start-offset', LaunchConfiguration('start_offset')
        ],
        output='screen'
    )

    # 5) sorrend: előbb RSP, 1s múlva save_odom, utána bag_play
    return LaunchDescription([
        bag_arg, skip_arg,
        rsp,
        TimerAction(period=1.0, actions=[save_odom]),
        TimerAction(period=2.0, actions=[bag_play]),
    ])
