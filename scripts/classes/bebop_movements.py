#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class BebopMovements:
    def __init__(self, pub, pub_takeoff, pub_land):
        self.pub = pub
        self.pub_takeoff = pub_takeoff
        self.pub_land = pub_land
        self.twist = Twist()

    def initial_takeoff(self):
        rospy.sleep(1)
        print('Taking off...')
        self.pub_takeoff.publish(Empty())
        rospy.sleep(2)

        # Setting zero.
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0

        # Setting level
        for i in range(3):
            self.twist.linear.z = 1
            rospy.sleep(0.2)
            self.pub.publish(self.twist)

        self.twist.linear.z = 0
        rospy.sleep(0.2)

    def landing(self):
        self.reset_twist()
        print('Landing...')
        rospy.sleep(0.1)
        self.pub.publish(self.twist)
        rospy.sleep(0.2)
        self.pub_land.publish(Empty())
        print('Successful landing')

    def forward(self):
        print('Going forward...')
        rospy.sleep(0.1)
        for _ in range(2):
            self.twist.linear.x = 1
            rospy.sleep(0.2)
            self.pub.publish(self.twist)
        self.twist.linear.x = 0

    def left(self):
        print('Going left...')
        rospy.sleep(0.1)
        for _ in range(1):
            self.twist.linear.y = 1
            rospy.sleep(0.4)
            self.pub.publish(self.twist)
        self.twist.linear.y = 0

    def right(self):
        print('Going right...')
        rospy.sleep(0.1)
        for _ in range(1):
            self.twist.linear.y = -1
            rospy.sleep(0.4)
            self.pub.publish(self.twist)
        self.twist.linear.y = 0

    def backwards(self):
        print('Going backwards')
        rospy.sleep(0.1)
        for _ in range(3):
            self.twist.linear.x = -1
            rospy.sleep(0.4)
            self.pub.publish(self.twist)
        self.twist.linear.x = 0

    def down(self):
        print('Going down...')
        rospy.sleep(0.1)
        for _ in range(2):
            self.twist.linear.z = -1
            rospy.sleep(0.4)
            self.pub.publish(self.twist)
        self.twist.linear.z = 0

    def up(self):
        print('Going up...')
        rospy.sleep(0.1)
        for _ in range(2):
            self.twist.linear.z = 1
            rospy.sleep(0.4)
            self.pub.publish(self.twist)
        self.twist.linear.z = 0

    def turn_left(self):
        print('Turning left...')
        rospy.sleep(0.1)
        for _ in range(2):
            self.twist.angular.z = 1
            rospy.sleep(0.2)
            self.pub.publish(self.twist)
        self.twist.angular.z = 0

    def turn_right(self):
        print('Turning right...')
        rospy.sleep(0.1)
        for _ in range(2):
            self.twist.angular.z = -1
            rospy.sleep(0.2)
            self.pub.publish(self.twist)
        self.twist.angular.z = 0

    def reset_twist(self):
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
