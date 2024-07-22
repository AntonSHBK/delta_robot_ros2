import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Название пакета
    pkg_name = 'delta_robot_ros2'
    
    # Директория пакета
    pkg_share = get_package_share_directory(pkg_name)
    
    # Пространство имён
    robot_namespace = 'delta_robot'
    

    # # Путь к файлу .xacro
    # urdf_file = os.path.join(pkg_share, 'urdf', 'delta_robot.xacro')    
    
    # # Путь к файлу конфигурации RViz
    # rviz_config_file = os.path.join(pkg_share, 'rviz', 'base.rviz')
    # # Путь к файлу конфигурации контроллеров
    # controllers_yaml = os.path.join(pkg_share, 'config', 'delta_robot_controllers.yml')
    
    
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(pkg_name),
                    "urdf",
                    "delta_robot.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(pkg_name),
            "config",
            "delta_robot_controllers.yaml",
        ]
    )
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(pkg_name), "rviz", "base.rviz"]
    )    
    
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false', 
            description='Use simulation time if true'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    
    # Initialize Arguments
    # use_sim_time = LaunchConfiguration("use_sim_time")
    # robot_description = LaunchConfiguration("robot_description")
    # controllers_config = LaunchConfiguration("controllers_config")
    gui = LaunchConfiguration("gui")
    
    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_namespace,  # Добавление пространства имен
        output='screen',
        parameters=[robot_description]
    )
    
    # joint_state_publisher также в пространстве имен
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=robot_namespace,  # Добавление пространства имен
        output='screen'
    )

    # Контроллеры
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='ros2_control_node',
        namespace=robot_namespace,  # Добавление пространства имен
        output='screen',
        parameters=[
            robot_controllers,
            {
                
            }],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    
    # joint_state_broadcaster_node = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     name='joint_state_broadcaster_spawner',
    #     namespace=robot_namespace,  # Добавление пространства имен
    #     output='screen',
    #     arguments=['joint_state_broadcaster', '--controller-manager', 
    #                 f'/{robot_namespace}/controller_manager',
    #                 ],
    # )
    
    # robot_controller_node = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     name='position_controllers_spawner',
    #     namespace=robot_namespace,  # Добавление пространства имен
    #     output='screen',
    #     arguments=['position_controllers', '--controller-manager',  
    #                f'/{robot_namespace}/controller_manager',
    #                ],
    # )

    # Нода управления
    # delta_control_node = Node(
    #     package='delta_robot_ros2',
    #     executable='delta_robot_controller.py',
    #     name='delta_robot_controller',
    #     namespace=robot_namespace,
    #     output='screen'
    # ),

    # RViz запускается в том же пространстве имен
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=robot_namespace,  # Добавление пространства имен
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(gui),
    )
    
    # # gazebo
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
    #     )
    # )
    
    # gz_node = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     output="screen",
    #     arguments=[
    #         "-topic", f"/{robot_namespace}/robot_description",
    #         "-name", "delta_robot",
    #     ],
    # )
    
    # # Delay rviz start after `joint_state_broadcaster`
    # delay_rviz_after_joint_state_broadcaster_node = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_node,
    #         on_exit=[rviz_node],
    #     )
    # )

    # # Delay start of robot_controller after `joint_state_broadcaster`
    # delay_robot_controller_node_after_joint_state_broadcaster_node = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_node,
    #         on_exit=[robot_controller_node],
    #     )
    # )

    # delay_gz_node_after_joint_state_broadcaster_node = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_node,
    #         on_exit=[gz_node],
    #     )
    # )
    
    # nodes = [
    #     robot_state_publisher_node,
    #     ros2_control_node,
    #     joint_state_broadcaster_node,
    #     delay_rviz_after_joint_state_broadcaster_node,
    #     delay_robot_controller_node_after_joint_state_broadcaster_node,
    #     delay_gz_node_after_joint_state_broadcaster_node``
    # ]   
    
    nodes = [
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_publisher_node,
        # joint_state_broadcaster_node,
        # robot_controller_node,
        rviz_node,
        # gazebo,
        # gz_node
    ] 

    return LaunchDescription(declared_arguments + nodes)
