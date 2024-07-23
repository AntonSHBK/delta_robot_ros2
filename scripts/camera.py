#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageConverter:
    def __init__(self):
        # Publisher and subscriber initialization
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)
        self.bridge = CvBridge()
        self.raw_image_sub = rospy.Subscriber("/camera/image_raw", Image, self.img_callback)

    def callback(self, data):
        # Converting image from ROS format to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        # Drawing a circle on the image if the size is sufficient
        (rows, cols, channels) = cv_image.shape
        if cols > 60 and rows > 60:
            cv2.circle(cv_image, (50, 50), 10, 255)

        # Displaying the processed image
        cv2.imshow("Image window", cv_image)
        cv2.resizeWindow('Image window', 400, 400)
        cv2.waitKey(3)

        # Publishing the modified image
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

    def img_callback(self, data):
        # Simple display of received images
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("Raw image", cv_image)
            cv2.waitKey(3)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

def main():
    rospy.init_node('camera_cv_converter_node', anonymous=True)
    ic = ImageConverter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
