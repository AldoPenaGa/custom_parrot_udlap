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
        self.reset_twist

    def initial_takeoff(self):
        print('Taking off...')
        self.pub_takeoff.publish(Empty())

        # Setting level
        for i in range(3):
            self.twist.linear.z = 1
            self.pub.publish(self.twist)

    def landing(self):
        self.reset_twist()
        print('Landing...')
        self.pub.publish(self.twist)
        self.pub_land.publish(Empty())
        print('Successful landing')

    def forward(self):
        print('Going forward...')
        for _ in range(2):
            self.twist.linear.x = 1
            self.pub.publish(self.twist)

    def left(self):
        print('Going left...')
        for _ in range(1):
            self.twist.linear.y = 1
            self.pub.publish(self.twist)

    def right(self):
        print('Going right...')
        for _ in range(1):
            self.twist.linear.y = -1
            self.pub.publish(self.twist)

    def backwards(self):
        print('Going backwards')
        for _ in range(2):
            self.twist.linear.x = -1
            self.pub.publish(self.twist)

    def down(self):
        print('Going down...')
        for _ in range(2):
            self.twist.linear.z = -1
            self.pub.publish(self.twist)

    def up(self):
        print('Going up...')
        for _ in range(2):
            self.twist.linear.z = 1
            self.pub.publish(self.twist)

    def turn_left(self):
        print('Turning left...')
        for _ in range(2):
            self.twist.angular.z = 1
            self.pub.publish(self.twist)

    def turn_right(self):
        print('Turning right...')
        for _ in range(2):
            self.twist.angular.z = -1
            self.pub.publish(self.twist)

    def reset_twist(self):
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
