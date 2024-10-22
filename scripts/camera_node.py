#!/usr/bin/env python
import rospy
import cv2
import os
import sys
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Import other classes
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(project_root)

from scripts.classes.camera_class import BebopCameraProcessor

class BebopCameraNode:
    def __init__(self):
        rospy.init_node('bebop_camera_node', anonymous=True)

        self.bridge = CvBridge()
        self.command_pub = rospy.Publisher('/bebop/command', String, queue_size=1)
        self.image_sub = rospy.Subscriber("/bebop/image_raw", Image, self.image_callback)

        self.processor = BebopCameraProcessor()

        rospy.spin()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process image using the camera_class processor
            processed_image, command = self.processor.process_image(cv_image)

            cv2.imshow("Bebop Camera", processed_image)
            cv2.waitKey(1)

            # Publish the command if any
            if command:
                rospy.loginfo(f"Command: {command}")
                self.command_pub.publish(command)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == '__main__':
    try:
        BebopCameraNode()
    except rospy.ROSInterruptException:
        pass
