# odom_logger/launch/save_odom.launch.py

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('odom_logger')

    # 1) Milyen topicról hallgatunk odometriát?
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Az Odometry topic neve'
    )
    # 2) Hova mentse a CSV-t (alapból: csomag/logged_file/real_traj.csv)
    default_csv = os.path.join(pkg_share, 'logged_file', 'real_traj.csv')
    output_arg = DeclareLaunchArgument(
        'output_file',
        default_value=default_csv,
        description='CSV fájl teljes elérési útvonala'
    )
    # 3) Használjunk-e sim_time-et
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='use_sim_time paraméter'
    )

    save_node = Node(
        package='odom_logger',
        executable='save_odom',
        name='save_odom',
        output='screen',
        # paraméterek a node-hoz
        parameters=[{
            'odom_topic': LaunchConfiguration('odom_topic'),
            'output_file': LaunchConfiguration('output_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        # remapeljük a témát, ha pl. más névre kell hallgatnia
        remappings=[
            ('/odom', LaunchConfiguration('odom_topic'))
        ]
    )

    return LaunchDescription([
        odom_topic_arg,
        output_arg,
        sim_time_arg,
        # (ha sim_time=true, szükség lehet a CLOCK topic exportjára)
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        save_node,
    ])
