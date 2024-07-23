# Delta Robot ROS 2 Simulation

## Already in work!!!

## Project Overview

This project is aimed at simulating a Delta Robot in ROS 2 using `ros2_control` for managing the robot's movements. The robot is defined using URDF and Xacro files, and its movements are visualized using RViz and Gazebo. The primary goal is to create a robust simulation environment for a Delta Robot and develop a control node to manage the actuators of the robot's legs.

## Features

- **URDF/Xacro Model**: Detailed robot description using URDF and Xacro.
- **ROS 2 Control Integration**: Utilizes `ros2_control` for actuator control.
- **Dynamic Visualization**: Real-time visualization in RViz and Gazebo.
- **Inverse Kinematics**: Custom Python node for inverse kinematics and motor control.

## Getting Started

### Prerequisites

- ROS 2 Humble
- Gazebo
- Python 3

### Installation

1. **Clone the repository**:
    ```sh
    git clone https://github.com/AntonSHBK/delta_robot_ros2.git
    cd delta_robot_ros2
    ```

2. **Build the package**:
    ```sh
    colcon build --packages-select delta_robot_ros2
    ```

3. **Source the setup file**:
    ```sh
    source install/setup.bash
    ```

### Running the Simulation

1. **Launch the robot description in RViz**:
    ```sh
    ros2 launch delta_robot_ros2 display.launch.py
    ```
    ```

### Project Structure

- `urdf/`: Contains URDF and Xacro files defining the robot.
- `launch/`: Contains launch files for starting the simulation and visualization.
- `src/`: Source files for the custom control node.

## Useful Resources

- **ROS 2 Documentation**: [https://docs.ros.org/en/foxy/index.html](https://docs.ros.org/en/foxy/index.html)
- **ros2_control Documentation**: [https://control.ros.org/](https://control.ros.org/)
- **Gazebo Documentation**: [http://gazebosim.org/tutorials](http://gazebosim.org/tutorials)
- **TF2 Documentation**: [http://wiki.ros.org/tf2](http://wiki.ros.org/tf2)

## Contacts

For questions or support, please contact:

- **Name**: Pisarenko Anton
- **Email**: anton42@yandex.ru
- **Telegram**: [antonSHBK](https://t.me/antonSHBK)

## Citation

If you use this work in your research, please cite it as follows:

```latex
@misc{yourname2024deltarobot,
  author = {Pisarenko Anton},
  title = {Delta Robot ROS 2 Simulation},
  year = {2024},
  publisher = {GitHub},
  journal = {GitHub repository},
  howpublished = {\url{https://github.com/AntonSHBK/delta_robot_ros2}},
}
```