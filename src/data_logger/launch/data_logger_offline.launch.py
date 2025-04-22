from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.events import Shutdown
from launch.actions import EmitEvent

def generate_launch_description():
    # Alapértelmezett értékek
    default_bag_path = '/home/ezo/f1tenth_ws/bags/rosbag2_2025_03_18-15_37_52'
    default_output_dir = '/home/ezo/f1tenth_ws/src/data_logger/logged_file/raw'

    # Paraméterek
    bag_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=TextSubstitution(text=default_bag_path),
        description='Path to rosbag2 folder'
    )

    output_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=TextSubstitution(text=default_output_dir),
        description='Output directory for CSV files'
    )

    # CSV fájlnevek
    odom_csv = PathJoinSubstitution([LaunchConfiguration('output_dir'), 'odom.csv'])
    imu_csv = PathJoinSubstitution([LaunchConfiguration('output_dir'), 'imu.csv'])
    js_csv = PathJoinSubstitution([LaunchConfiguration('output_dir'), 'joint_states.csv'])
    ackermann_csv = PathJoinSubstitution([LaunchConfiguration('output_dir'), 'ackermann_cmd.csv'])
    drive_csv = PathJoinSubstitution([LaunchConfiguration('output_dir'), 'drive.csv'])

    # Logger node-ok
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

    # /ackermann_cmd loggolása
    ackermann_logger = Node(
        package='data_logger',
        executable='save_ackermann_cmd',
        name='save_ackermann_cmd',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'ackermann_cmd_topic': '/ackermann_cmd',
            'output_file': ackermann_csv
        }]
    )

    # /drive loggolása külön fájlba
    drive_logger = Node(
        package='data_logger',
        executable='save_ackermann_cmd',
        name='save_drive_cmd',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'ackermann_cmd_topic': '/drive',
            'output_file': drive_csv
        }]
    )

    # ros2 bag lejátszása
    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            LaunchConfiguration('bag_path'),
            '--clock'
        ],
        output='screen'
    )

    # Shutdown action
    # A rosbag hossza ~177 másodperc, ezért 180 mp után automatikusan leállítjuk a folyamatot
    shutdown_after = TimerAction(
        period=180.0,
        actions=[EmitEvent(event=Shutdown())]
    )

    return LaunchDescription([
        bag_arg,
        output_arg,
        TimerAction(period=0.5, actions=[
            odom_logger, imu_logger, joint_states_logger, ackermann_logger, drive_logger
        ]),
        TimerAction(period=1.0, actions=[bag_play]),
        shutdown_after
    ])
