cmake_minimum_required(VERSION 3.8)
project(delta_robot_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(urdf REQUIRED)
# find_package(xacro REQUIRED)
# find_package(robot_state_publisher REQUIRED)
# find_package(rviz2 REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(visualization_msgs REQUIRED)
# find_package(python_cmake_module REQUIRED)  # Add python cmake module
# find_package(Python3 REQUIRED COMPONENTS Interpreter)  # Find Python interpreter

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  urdf
  xacro
  robot_state_publisher
  rviz2
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  python_cmake_module
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Install Python scripts
install(PROGRAMS
  scripts/delta_robot_controller.py
  scripts/kinematic.py
  scripts/dinamic_tf.py
  scripts/user_gui.py
  scripts/camera.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install urdf files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install urdf files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz files
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
