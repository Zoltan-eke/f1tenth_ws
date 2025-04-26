cmake_minimum_required(VERSION 3.8)
project(car_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Kötelező build tool
find_package(ament_cmake REQUIRED)

# --- Launchhoz és URDF-hez használt csomagok (futtatási időben kellenek)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(ackermann_steering_controller REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(xacro REQUIRED)

# --- Launch/config/urdf mappák telepítése
install(DIRECTORY
  launch
  config
  urdf
  bags
  DESTINATION share/${PROJECT_NAME}
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
