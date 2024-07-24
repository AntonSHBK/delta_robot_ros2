#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point

from kinematic import DeltaRobot

class DeltaRobotController(Node):
    def __init__(self):
        super().__init__('delta_robot_controller')
        
        # Создаем экземпляр DeltaRobot с вашими параметрами
        self.robot = DeltaRobot(
            base_radius=500, 
            platform_radius=300, 
            upper_arm_length=400, 
            forearm_length=300
            )
        
        # Подписка на топик с целевыми координатами
        self.subscription = self.create_subscription(
            Point,
            'target_position',
            self.target_position_callback,
            1000
        )
        
        # Издатель для отправки команд контроллеру
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            1000
        )
        
        self.get_logger().info('Delta Robot Controller has been started.')
    
    def target_position_callback(self, msg):
        target_position = [msg.x, msg.y, msg.z]
        try:
            joint_angles = self.robot.inverse_kinematics(target_position)
            self.publish_joint_commands(joint_angles)
        except ValueError as e:
            self.get_logger().error(f'Error in inverse kinematics: {e}')
    
    def publish_joint_commands(self, joint_angles):
        command_msg = Float64MultiArray()
        command_msg.data = joint_angles
        self.publisher.publish(command_msg)
        self.get_logger().info(f'Published joint commands: {joint_angles}')

def main(args=None):
    rclpy.init(args=args)
    node = DeltaRobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
