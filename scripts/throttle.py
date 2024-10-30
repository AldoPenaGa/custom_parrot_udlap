#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class ThrottleCommandNode:
    def __init__(self):

        rospy.init_node('throttle_command_node', anonymous=True)

        self.rate = rospy.Rate(0.5)

        self.sub = rospy.Subscriber("/bebop/command", String, self.command_callback)

        self.pub = rospy.Publisher("/bebop/command_throttled", String, queue_size=1)

        self.last_command = None

    def command_callback(self, msg):

        self.last_command = msg

    def start(self):

        while not rospy.is_shutdown():
            if self.last_command:
                self.pub.publish(self.last_command)  
            self.rate.sleep()  

if __name__ == '__main__':
    try:
        throttle_command_node = ThrottleCommandNode()
        throttle_command_node.start()
    except rospy.ROSInterruptException:
        pass