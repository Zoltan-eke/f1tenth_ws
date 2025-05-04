cmake_minimum_required(VERSION 3.8)
project(car_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Kötelező build tool
find_package(ament_cmake REQUIRED)


# --- Launchhoz és URDF-hez használt csomagok (futtatási időben kellenek)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(controller_manager REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(ackermann_steering_controller REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(xacro REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

# --- Launch/config/urdf mappák telepítése
install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}
)

### Telepítsük a Python scriptet is
install(PROGRAMS
    scripts/ackermann_to_odom.py
    scripts/drive_replay.py
    scripts/vehicle_model_node.py
    scripts/leader_position_publisher.py
    scripts/spawn_f1tenth_vehicle.py
    scripts/spawn_f1tenth_vehicle_debug.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
