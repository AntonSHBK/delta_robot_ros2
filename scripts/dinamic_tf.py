import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
import math

class TransformManager:
    def __init__(self, parent_frame, child_frame, namespace=""):
        self.namespace = namespace
        self.parent_frame = f"{namespace}/{parent_frame}" if namespace else parent_frame
        self.child_frame = f"{namespace}/{child_frame}" if namespace else child_frame
        self.transform = TransformStamped()
        self.transform.header.frame_id = self.parent_frame
        self.transform.child_frame_id = self.child_frame
        self.br = tf2_ros.TransformBroadcaster()

    def update_transform(self, translation, rotation):
        self.transform.header.stamp = rospy.Time.now()
        self.transform.transform.translation.x, self.transform.transform.translation.y, self.transform.transform.translation.z = translation
        self.transform.transform.rotation.x, self.transform.transform.rotation.y, self.transform.transform.rotation.z, self.transform.transform.rotation.w = rotation
        self.br.sendTransform(self.transform)

class Robot:
    def __init__(self, namespace=""):
        self.namespace = namespace
        self.link1 = TransformManager("base_link", "link1", namespace)
        self.link2 = TransformManager("link1", "link2", namespace)

    def update(self):
        # Simulation of dynamic movements
        x = math.sin(rospy.get_time())
        y = math.cos(rospy.get_time())
        rotation = tf_conversions.transformations.quaternion_from_euler(0, 0, rospy.get_time())
        self.link1.update_transform((x, 0, 0), rotation)
        self.link2.update_transform((0.5, y, 0), rotation)

if __name__ == '__main__':
    rospy.init_node('robot_tf_publisher')
    namespace = rospy.get_param('robot_namespace', '')  # Get namespace from ROS parameter server
    robot = Robot(namespace)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        robot.update()
        rate.sleep()
