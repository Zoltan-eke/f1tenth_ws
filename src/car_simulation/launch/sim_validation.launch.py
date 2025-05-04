# car_simulation/launch/sim_validation.launch.py
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler, EmitEvent, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit


WORKSPACE_ROOT  = os.path.expanduser('~/f1tenth_ws')
REAL_DIR        = os.path.join(WORKSPACE_ROOT, 'src', 'data_logger', 'logged_file', 'filtered')
SIM_DIR         = os.path.join(WORKSPACE_ROOT, 'src', 'data_logger', 'logged_file', 'simulated')
os.makedirs(SIM_DIR, exist_ok=True)

def generate_launch_description():

    pkg_share = FindPackageShare('car_description').find('car_description')
    params_file = PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])
    xacro_file  = os.path.join(pkg_share, 'urdf', 'base_link.xacro')

    # xacro-hoz launch-argok
    declare_wb = DeclareLaunchArgument('wheel_base', default_value='0.320',description='Tengelytáv [m]')
    declare_wr = DeclareLaunchArgument('wheel_radius', default_value='0.0534',description='Kerék sugár [m]')

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[
            params_file,
            {'robot_description': Command([
                'xacro ', xacro_file,
                ' wheel_base:=', LaunchConfiguration('wheel_base'),
                ' wheel_radius:=', LaunchConfiguration('wheel_radius'),
            ])}
        ],
    )


    drive_csv = os.path.join(SIM_DIR, 'sim_drive.csv')
    drive_replay = Node(
        package='car_simulation', executable='drive_replay.py',
        name='drive_replay', output='screen',
        parameters=[
            {'drive_file': drive_csv},
            {'topic':      '/drive'},
        ],
    )

    ack_to_odom = Node(
        package='car_simulation', executable='ackermann_to_odom.py',
        name='sim_vehicle', output='screen',
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[
            ('odom',  'sim_odom'),
            ('tf',    'sim_tf'),
        ],
    )

#    save_drive = Node(
#        package='data_logger', executable='save_ackermann_cmd',
#        name='save_sim_drive', output='screen',
#        parameters=[
#            {'output_dir':      SIM_DIR},
#            {'output_filename': 'sim_drive.csv'},
#        ],
#        remappings=[('ackermann_cmd', 'drive')],
#    )

    save_odom = Node(
        package='data_logger', executable='save_odom',
        name='save_sim_odom', output='screen',
        parameters=[
            {'output_dir':      SIM_DIR},
            {'output_filename': 'sim_odom.csv'},
        ],
        remappings=[('odom', 'sim_odom')],
    )

    save_tf = Node(
        package='data_logger', executable='save_tf',
        name='save_sim_tf', output='screen',
        parameters=[
            {'output_dir':      SIM_DIR},
            {'output_filename': 'sim_tf.csv'},
            {'use_sim_time': True},
        ],
        remappings=[('tf', 'sim_tf')],
    )

    shutdown_on_replay_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=drive_replay,
            on_exit=[ Shutdown() ],
        )
    )

    return LaunchDescription([
        shutdown_on_replay_exit,
        declare_wb, declare_wr,
        rsp,
        drive_replay,
        ack_to_odom,
        TimerAction(period=0.1, actions=[save_odom, save_tf,]),

        

    ])
