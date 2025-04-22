from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler,
    EmitEvent
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution,
    PathJoinSubstitution
)
from launch_ros.actions import Node


def generate_launch_description():
    # -----------------------------
    # 1. Launch arguments
    # -----------------------------
    # Path to the ros2 bag file (.db3) to play back
    # Default location: /home/ezo/f1tenth_ws/bags/rosbag2_2025_03_18-15_37_52
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=TextSubstitution(text='/home/ezo/f1tenth_ws/bags/rosbag2_2025_03_18-15_37_52'),
        description='Absolute path to the recorded ros2 bag (db3 file) to play back'
    )
    # Directory where CSV log files will be saved
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=TextSubstitution(text='/home/ezo/f1tenth_ws/src/data_logger/logged_file/raw'),
        description='Directory where CSV log files will be saved'
    )
    # Flag to enable simulation time (use /clock from bag playback)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Whether to use /clock from bag playback as ROS time'
    )

    # -----------------------------
    # 2. CSV output file paths
    # -----------------------------
    output_dir = LaunchConfiguration('output_dir')
    odom_csv = PathJoinSubstitution([output_dir, 'odom.csv'])
    imu_csv = PathJoinSubstitution([output_dir, 'imu.csv'])
    joint_states_csv = PathJoinSubstitution([output_dir, 'joint_states.csv'])
    drive_cmd_csv = PathJoinSubstitution([output_dir, 'drive.csv'])

    # -----------------------------
    # 3. Define Save_* logger nodes
    # -----------------------------
    odom_logger = Node(
        package='data_logger',
        executable='save_odom',
        name='save_odom',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'imu_topic': '/sensors/imu/raw',
            'output_file': imu_csv
        }]
    )

    joint_states_logger = Node(
        package='data_logger',
        executable='save_joint_states',
        name='save_joint_states',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'joint_states_topic': '/joint_states',
            'output_file': joint_states_csv
        }]
    )

    # Only log /drive Ackermann commands; duplicates removed
    drive_cmd_logger = Node(
        package='data_logger',
        executable='save_ackermann_cmd',
        name='save_drive_cmd',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'ackermann_cmd_topic': '/drive',
            'output_file': drive_cmd_csv
        }]
    )

    # -----------------------------
    # 4. Play back the bag file
    # -----------------------------
    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            LaunchConfiguration('bag_path'),
            '--clock'
        ],
        output='screen'
    )

    # -----------------------------
    # 5. Launch order & timing
    # -----------------------------
    # Start bag playback immediately
    start_bag = bag_play
    # Delay logger startup by 1 second to ensure /clock and topics are available
    start_loggers = TimerAction(
        period=1.0,
        actions=[
            odom_logger,
            imu_logger,
            joint_states_logger,
            drive_cmd_logger
        ]
    )

    # -----------------------------
    # 6. Automatic shutdown on bag completion
    # -----------------------------
    # When the bag_play process exits, trigger EmitEvent(Shutdown()) to stop all nodes
    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_play,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )

    # -----------------------------
    # Return LaunchDescription
    # -----------------------------
    return LaunchDescription([
        bag_path_arg,
        output_dir_arg,
        use_sim_time_arg,
        start_bag,
        start_loggers,
        shutdown_handler,
    ])