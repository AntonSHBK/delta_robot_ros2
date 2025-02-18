# Commands

Этот файл содержит команды для сборки и запуска симуляции дельта робота, а также для отправки команд на контроллеры и топики.

## Сборка проекта

```bash
colcon build --packages-select delta_robot_ros2
```
Эта команда выполняет сборку пакета `delta_robot_ros2` с использованием `colcon`, который является стандартным инструментом сборки для ROS 2.

## Запуск симуляции дельта робота

```bash
ros2 launch delta_robot_ros2 delta_robot_run.py
```
Эта команда запускает симуляцию дельта робота с использованием файла запуска `delta_robot_run.py`. Она включает в себя загрузку модели робота в RViz и/или Gazebo, а также запуск необходимых нод управления.

## Открытие менеджера контроллеров

```bash
ros2 run rqt_controller_manager rqt_controller_manager
```
Эта команда запускает графический интерфейс `rqt_controller_manager`, который позволяет управлять контроллерами робота. Вы можете включать и отключать контроллеры, а также настраивать их параметры.

## Публикация команд на контроллер позиций

```bash
ros2 topic pub /forward_position_controller/commands std_msgs/Float64MultiArray "data: [1, 1, 2]"
```
Эта команда публикует сообщение типа `Float64MultiArray` на топик `/forward_position_controller/commands`. Данные в сообщении представляют собой целевые позиции для контроллера позиций робота. В данном примере задаются позиции [1, 1, 2].

## Публикация целевой позиции для обратной кинематики

```bash
ros2 topic pub /target_position geometry_msgs/Point "{x: 0.1, y: 0.2, z: -0.3}"
```
Эта команда публикует сообщение типа `Point` на топик `/target_position`. Сообщение содержит целевые координаты (x, y, z) для дельта робота. В данном примере задаются координаты {x: 0.1, y: 0.2, z: -0.3}, которые будут использоваться для расчета обратной кинематики.
