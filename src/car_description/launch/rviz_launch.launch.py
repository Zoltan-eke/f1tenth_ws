import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'car_description'
    urdf_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'base_link.xacro')
    rviz_config_file = os.path.join(get_package_share_directory(pkg_name), 'rviz', 'racecar.rviz')
    controller_params_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'ackermann_control.yaml')
    robot_params_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'car_params.yaml')

    # --- Paraméterek definiálása (LaunchConfiguration) ---
    #  Ezek most *opcionálisak*, mert az alapértelmezett értékek a car_params.yaml-ben vannak.
    #  Ha *felül* akarod bírálni az alapértelmezett értékeket, akkor használd ezeket.
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius', default_value='0.05',
        description='Radius of the wheels'
    )

    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation', default_value='0.32',
        description='Separation between the wheels'
    )

    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase', default_value='0.235',
        description='Wheelbase of the car'
    )
    wheel_width_arg = DeclareLaunchArgument(
        'wheel_width', default_value='0.045',
        description='width of the wheels'
    )
    wheel_mass_arg = DeclareLaunchArgument(
        'wheel_mass', default_value='0.3',
        description='Mass of wheels'
    )
    chassis_mass_arg = DeclareLaunchArgument(
        'chassis_mass', default_value='1.0',
        description='Mass of chassis'
    )
    steering_hinge_mass_arg = DeclareLaunchArgument(
        'steering_hinge_mass', default_value='0.1',
        description='Mass of steering hinges'
    )
    # --- robot_state_publisher node (xacro feldolgozása Command-dal) ---
    robot_description = ParameterValue(Command([
        'xacro ', urdf_file,
        ' wheel_radius:=', LaunchConfiguration('wheel_radius'),
        ' wheel_separation:=', LaunchConfiguration('wheel_separation'),
        ' wheelbase:=', LaunchConfiguration('wheelbase'),
        ' wheel_width:=', LaunchConfiguration('wheel_width'),
        ' wheel_mass:=', LaunchConfiguration('wheel_mass'),
        ' chassis_mass:=', LaunchConfiguration('chassis_mass'),
        ' steering_hinge_mass:=', LaunchConfiguration('steering_hinge_mass')

    ]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}]
    )

   # --- RViz2 node ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # --- ros2_control node-ok (spawner-ek) ---
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file,  # Általános konfiguráció
                    robot_params_file,     # Robot-specifikus paraméterek
                    #{'robot_description': robot_description}
                    ],  # Robot leírás
        remappings=[
            ("~/robot_description", "/robot_description"), ],
        output="screen",
    )

    # Késleltetett indítás, hogy a controller_manager biztosan fusson, mielőtt a spawner-ek elindulnak.
    # Ez NEM a legjobb megoldás (jobb lenne condition-t használni), de a legegyszerűbb.
    delay_for_spawners = ExecuteProcess(
        cmd=['sleep', '5'],
        output='screen'
    )

    ackermann_steering_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    joint_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    ## --- teleop_twist_keyboard node ---
    #teleop_twist_keyboard_node = Node(
    #    package='teleop_twist_keyboard',
    #    executable='teleop_twist_keyboard',
    #    name='teleop_twist_keyboard',
    #    output='screen',
    #    remappings=[
    #        ('/cmd_vel', '/my_ackermann/cmd_vel')  # Ellenőrizd, hogy ez a helyes topic név!
    #    ],
    #    parameters=[
    #        {'scale_linear.x': 1.0},  # Ezeket is át lehetne adni a YAML-ből, ha akarod
    #        {'scale_angular.z': 0.5},
    #    ]
    #)

    delayed_ackermann_steering_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=delay_for_spawners,
            on_exit=[ackermann_steering_spawner],
        )
    )
    delayed_joint_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=delay_for_spawners,
            on_exit=[joint_broadcaster_spawner],
        )
    )

    return LaunchDescription([
        wheel_radius_arg,         # Ezek most már opcionálisak!
        wheel_separation_arg,
        wheelbase_arg,
        wheel_width_arg,
        wheel_mass_arg,
        chassis_mass_arg,
        steering_hinge_mass_arg,
        robot_state_publisher_node,
        rviz_node,
        controller_manager_node,
        delay_for_spawners,
        delayed_ackermann_steering_spawner,
        delayed_joint_broadcaster_spawner,
        #teleop_twist_keyboard_node
    ])
