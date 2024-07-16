from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    pkg_share = get_package_share_directory('delta_robot_ros2')

    # Путь к файлу .xacro
    urdf_file = os.path.join(pkg_share, 'urdf', 'delta_robot.xacro')
    
    # Путь к файлу конфигурации RViz
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'base.rviz')

    # Путь к файлу конфигурации контроллеров
    controllers_yaml = os.path.join(pkg_share, 'config', 'delta_robot_controllers.yaml')
    
    # Пространство имён
    robot_namespace = 'robot1'

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='false', description='Use simulation time if true'),
        DeclareLaunchArgument(name='robot_description', default_value=Command(['xacro ', urdf_file]), description='URDF XML'),
        DeclareLaunchArgument(name='controllers_config', default_value=controllers_yaml, description='Controllers configuration YAML'),

        # Параметры модели загружены в сервер параметров в пространстве имен робота
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_namespace,  # Добавление пространства имен
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': LaunchConfiguration('robot_description')
            }]
        ),
        
        # joint_state_publisher также в пространстве имен
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=robot_namespace,  # Добавление пространства имен
            output='screen'
        ),

        # Контроллеры
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='ros2_control_node',
            namespace=robot_namespace,  # Добавление пространства имен
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': LaunchConfiguration('robot_description'),
                'controllers': LaunchConfiguration('controllers_config')
            }]
        ),

        # Нода управления
        # Node(
        #     package='delta_robot_ros2',
        #     executable='delta_robot_controller.py',
        #     name='delta_robot_controller',
        #     namespace=robot_namespace,
        #     output='screen'
        # ),

        # RViz запускается в том же пространстве имен
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=robot_namespace,  # Добавление пространства имен
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
        
        

        # Добавление динамического публикатора трансформаcdций для robot1
        # Node(
        #     package='your_package',
        #     executable='your_script.py',
        #     name='tf_publisher1',
        #     namespace='robot1',
        #     parameters=[{'robot_ns': 'robot1', 'base_frame': 'base_link', 'link_frame': 'link1'}],
        #     output='screen'
        # )

        # Добавление динамического публикатора трансформаций для robot1
        # Node(
        #     package='your_package',
        #     executable='your_script.py',
        #     name='tf_publisher1',
        #     namespace='robot1',
        #     parameters=[{'robot_ns': 'robot1', 'base_frame': 'base_link', 'link_frame': 'link1'}],
        #     output='screen'
        # )
    ])
