#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Elérési út a model fájlhoz a car_description csomagból
    xacro_path = PathJoinSubstitution([
        FindPackageShare('car_description'),
        'urdf',
        'base_link.xacro'
    ])

    # robot_state_publisher node, a model betöltéséhez
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro', ' ', xacro_path]),
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        TimerAction(period=0.5, actions=[robot_state_publisher])
    ])
