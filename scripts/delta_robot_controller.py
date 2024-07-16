#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, TransformStamped
import tf
import tf2_ros
import numpy as np

from kinematic import DeltaRobot

class DeltaRobotController:
    def __init__(self):
        rospy.init_node('delta_robot_controller')

        # Подписка на топик с командами
        self.command_sub = rospy.Subscriber('/delta_robot/command', Pose, self.command_callback)
        
        # Публикация углов для ног
        self.leg1_joint1_pub = rospy.Publisher('/delta_robot/leg_1_joint_1_controller/command', 
        Float64, queue_size=10)
        self.leg2_joint1_pub = rospy.Publisher('/delta_robot/leg_2_joint_1_controller/command', Float64, queue_size=10)
        self.leg3_joint1_pub = rospy.Publisher('/delta_robot/leg_3_joint_1_controller/command', Float64, queue_size=10)

        self.br = tf2_ros.TransformBroadcaster()

        base_radius = 500
        platform_radius = 100
        upper_arm_length = 400
        forearm_length = 300

        self.delta_robot = DeltaRobot(base_radius, platform_radius, 
                                      upper_arm_length, forearm_length)

    def command_callback(self, msg):
        # Обработка входной команды и расчет углов для ног
        target_position = np.array([msg.position.x, msg.position.y, msg.position.z])

        # Здесь будет расчет обратной кинематики

        leg_angles = self.delta_robot.inverse_kinematics(target_position)

        # Публикация углов для ног
        self.leg1_joint1_pub.publish(Float64(leg_angles[0]))
        self.leg2_joint1_pub.publish(Float64(leg_angles[1]))
        self.leg3_joint1_pub.publish(Float64(leg_angles[2]))

        # Публикация текущей трансформации для визуализации
        self.publish_transform('base_link', 'platform', target_position)

    def publish_transform(self, parent_frame, child_frame, translation):
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.br.sendTransform(t)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    controller = DeltaRobotController()
    controller.spin()
