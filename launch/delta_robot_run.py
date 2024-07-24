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
    # robot_namespace = 'delta_robot'   
    
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
    gui = LaunchConfiguration("gui")
    
    #  # gazebo
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
    #     )
    # )
    
    launch_description = [
        # gazebo
    ]
    
    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        # namespace=robot_namespace,  # Добавление пространства имен
        output='both',
        parameters=[robot_description]
    )
    
    # joint_state_publisher также в пространстве имен
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     # namespace=robot_namespace,  # Добавление пространства имен
    #     output='both'
    # )

    # Контроллеры
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", f"/robot_description"),
        ],
    )
    
    joint_state_broadcaster_node = Node(
        package='controller_manager',
        executable='spawner',
        output='both',
        arguments=['joint_state_broadcaster', '--controller-manager', 
                    f'/controller_manager',
                    ],
    )
    
    forward_position_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        output='both',
        arguments=['forward_position_controller', '--controller-manager',  
                   f'/controller_manager',
                   ],
    )
    
    # position_controllers_node = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     output='both',
    #     arguments=['position_controller', '--controller-manager',  
    #                f'/controller_manager',
    #                ],
    # )
    
    delta_robot_controller_node = Node(
        package=pkg_name,
        executable='delta_robot_controller.py',
        name='delta_robot_controller',
        output='screen'
    )
    
    dinamic_tf_controller_node = Node(
        package=pkg_name,
        executable='dinamic_tf.py',
        name='dinamic_tf',
        output='screen'
    )
    
    delta_robot_controller_gui_node = Node(
        package=pkg_name,
        executable='user_gui.py',
        name='delta_robot_controller_gui',
        output='screen'
    )


    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(gui),
    )
    
    # gz_node = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     output="screen",
    #     arguments=[
    #         "-topic", f"/robot_description",
    #         "-name", "delta_robot",
    #     ],
    # )
    
    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_node,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_forward_controller_node_after_joint_state_broadcaster_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_node,
            on_exit=[forward_position_controller_node],
        )
    )
    
    # delay_position_controller_node_after_joint_state_broadcaster_node = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_node,
    #         on_exit=[position_controllers_node],
    #     )
    # )

    # delay_gz_node_after_joint_state_broadcaster_node = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_node,
    #         on_exit=[gz_node],
    #     )
    # )
    
    nodes = [
        robot_state_publisher_node,
        ros2_control_node,
        delta_robot_controller_node,
        dinamic_tf_controller_node,
        delta_robot_controller_gui_node,
        joint_state_broadcaster_node,
        delay_rviz_after_joint_state_broadcaster_node,
        delay_forward_controller_node_after_joint_state_broadcaster_node,
        # delay_position_controller_node_after_joint_state_broadcaster_node,
        # delay_gz_node_after_joint_state_broadcaster_node
    ]   

    return LaunchDescription(declared_arguments + launch_description + nodes)
