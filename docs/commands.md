# Commands

```bush

colcon build --packages-select delta_robot_ros2
colcon build --packages-select controlko_bringup

ros2 launch delta_robot_ros2 delta_robot_run.py

ros2 launch moveit_setup_assistant setup_assistant.launch.py

ros2 topic echo /fmu/out/vehicle_status

ros2 run rqt_controller_manager rqt_controller_manager

ros2 topic pub /forward_position_controller/commands std_msgs/Float64MultiArray "data: [1, 1, 2]"
ros2 topic pub /target_position geometry_msgs/Point "{x: 0.1, y: 0.2, z: -0.3}"

```
