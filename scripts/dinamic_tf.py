#!/usr/bin/env python3

import math
from time import sleep

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np


class DynamicPlatformUpdater(Node):
    def __init__(self):
        super().__init__('dynamic_platform_updater')

        # Подписка на топик с состояниями углов
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            100
        )

        # Broadcaster для TF
        self.br = tf2_ros.TransformBroadcaster(self)

        # Параметры робота (здесь нужно использовать те же параметры, что и в классе DeltaRobot)
        self.base_radius = 0.5
        self.platform_radius = 0.3
        self.upper_arm_length = 1.0
        self.forearm_length = 1.2
        
        self.circle_counter = 0.
        self.circke_step = 0.2

        self.get_logger().info('Dynamic Platform Updater has been started.')

    def joint_states_callback(self, msg):
        joint_positions = dict(zip(msg.name, msg.position))

        # Получение положений шарниров
        leg_1_joint_1 = joint_positions.get('leg_1_joint_1', 0.0)
        leg_2_joint_1 = joint_positions.get('leg_2_joint_1', 0.0)
        leg_3_joint_1 = joint_positions.get('leg_3_joint_1', 0.0)

        # Публикация трансформации
        x = math.cos(self.circle_counter) * self.circke_step
        y = math.sin(self.circle_counter) * self.circke_step
        z = 0.6
        
        self.circle_counter += 0.1
        sleep(0.1)
        
        self.publish_transform('base_link', 'platform', [x, y, z])
        
    def publish_transform(self, parent_frame, child_frame, position, rotation=(0., 0., 0.)):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]

        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicPlatformUpdater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

