#!/usr/bin/env python3

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

        self.get_logger().info('Dynamic Platform Updater has been started.')

    def joint_states_callback(self, msg):
        joint_positions = dict(zip(msg.name, msg.position))

        # Получение положений шарниров
        leg_1_joint_1 = joint_positions.get('leg_1_joint_1', 0.0)
        leg_2_joint_1 = joint_positions.get('leg_2_joint_1', 0.0)
        leg_3_joint_1 = joint_positions.get('leg_3_joint_1', 0.0)

        # Публикация трансформации
        self.publish_transform([0., 1., 2.])
        print(leg_1_joint_1, leg_2_joint_1, leg_3_joint_1)



    def publish_transform(self, platform_position):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'platform'

        t.transform.translation.x = platform_position[0]
        t.transform.translation.y = platform_position[1]
        t.transform.translation.z = platform_position[2]

        # Поворот подвижной платформы (если необходим)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
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

