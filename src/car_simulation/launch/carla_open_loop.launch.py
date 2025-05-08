#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('car_simulation')
    param_file = os.path.join(pkg_share, 'config', 'vehicle_params.yaml')
    log_dir = os.path.join(pkg_share, 'sim_log')


    # Leader jármű spawn
    LeaderSpawnNode = Node(
            package='car_simulation',
            executable='spawn_leader_vehicle',
            name='spawn_leader',
            output='screen',
        )
    
    # Leader pozíció publikálása
    LeaderPositionPublisherNode = Node(
            package='car_simulation',
            executable='leader_position_publisher',
            name='leader_pose_pub',
            output='screen',
        )

    # Open-loop parancs generálása a leader alapján
    OpenLoopPublisherNode = Node(
            package='car_simulation',
            executable='open_loop_cmd_publisher',
            name='open_loop_cmd_pub',
            output='screen',
            parameters=[param_file,] # Paraméter fájl betöltése
        )

    # Vehicle model node (validálandó)
    VehicleModelNode = Node(
            package='car_simulation',
            executable='vehicle_model_node',
            name='vehicle_model',
            output='screen',
            parameters=[param_file,], # Paraméter fájl betöltése
        )
    
    # Logger Node
    SimulationLogger = Node(
            package='car_simulation',
            executable='simulation_logger',
            name='simulation_logger',
            output='screen',
            parameters=[param_file,{'output_dir': log_dir}] # Paraméter fájl betöltése
        )

    return LaunchDescription([
        LeaderSpawnNode,
        LeaderPositionPublisherNode,
        OpenLoopPublisherNode,
        TimerAction(period=3.0,actions=[VehicleModelNode]),
        SimulationLogger
    ])
