#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

sleep = 0.5

class BebopMovements:
    def __init__(self, pub, pub_takeoff, pub_land):
        self.pub = pub
        self.pub_takeoff = pub_takeoff
        self.pub_land = pub_land
        self.twist = Twist()

    def initial_takeoff(self):
        rospy.sleep(1)
        print('\n Taking off...')
        self.pub_takeoff.publish(Empty())
        rospy.sleep(3)
        self.reset_twist()

    def landing(self):
        self.reset_twist()
        print('\n Landing...')
        rospy.sleep(sleep)
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.pub_land.publish(Empty())
        print('\n Successful landing')
        self.reset_twist()

    def forward(self):
        rospy.sleep(sleep)
        print('\n Going forward...')
        for _ in range(1):
            self.twist.linear.x = 0.5
            self.pub.publish(self.twist)
            rospy.sleep(sleep)
        self.reset_twist()

    def left(self):
        print('\n Going left...')
        rospy.sleep(sleep)
        for _ in range(1):
            self.twist.linear.y = 0.5
            self.pub.publish(self.twist)
            rospy.sleep(sleep)
        self.reset_twist()

    def right(self):
        rospy.sleep(sleep)
        print('\n Going right...')
        for _ in range(1):
            self.twist.linear.y = -0.5
            self.pub.publish(self.twist)
            rospy.sleep(sleep)
        self.reset_twist()

    def backwards(self):
        print('\n Going backwards')
        rospy.sleep(sleep)
        for _ in range(1):
            self.twist.linear.x = -0.5
            self.pub.publish(self.twist)
            rospy.sleep(sleep)
        self.reset_twist()

    def down(self):
        print('\n Going down...')
        rospy.sleep(sleep)
        for _ in range(1):
            self.twist.linear.z = -0.5
            self.pub.publish(self.twist)
            rospy.sleep(sleep)
        self.reset_twist()

    def up(self):
        print('\n Going up...')
        rospy.sleep(sleep)
        for _ in range(2):
            self.twist.linear.z = 0.5
            self.pub.publish(self.twist)
            rospy.sleep(sleep)
        self.reset_twist()

    def turn_left(self):
        print('\n Turning left...')
        rospy.sleep(sleep)
        for _ in range(1):
            self.twist.angular.z = 0.5
            self.pub.publish(self.twist)
            rospy.sleep(sleep)
        self.reset_twist()

    def turn_right(self):
        print('\n Turning right...')
        rospy.sleep(sleep)
        for _ in range(1):
            self.twist.angular.z = -0.5
            self.pub.publish(self.twist)
            rospy.sleep(sleep)
        self.reset_twist()

    def reset_twist(self):
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0