#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

sleep = 0.5

class BebopMovements:
    
    def __init__(self, pub, pub_takeoff, pub_land, pub_camera):
        self.pub = pub
        self.pub_takeoff = pub_takeoff
        self.pub_land = pub_land
        self.pub_camera = pub_camera
        self.twist = Twist()

    def initial_takeoff(self, mode_flag):

        if mode_flag not in ['automatic', 'teleop'] or rospy.is_shutdown():
            print('\n Invalid state')
            return
        rospy.sleep(1)

        print('\n Taking off...')
        self.pub_takeoff.publish(Empty())
        rospy.sleep(3)
        self.up(mode_flag)
        self.up(mode_flag)
        self.up(mode_flag)
        self.turn_right(mode_flag)
        rospy.sleep(1)
        self.reset_twist()


    def landing(self, mode_flag):
        if mode_flag not in ['automatic', 'teleop'] or rospy.is_shutdown():
            print('\n Invalid state')
            return
        self.reset_twist()
        print('\n Landing...')

        rospy.sleep(sleep)
        self.pub.publish(self.twist)
        rospy.sleep(sleep)

        self.pub_land.publish(Empty())
        
        print('\n Land done')
        self.reset_twist()

    def forward(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movement interrupted')
            return
        rospy.sleep(sleep)
        print('\n Going forward...')

        self.twist.linear.x = 1
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        # self.twist.linear.x = 1       All these for passing the windows
        # self.pub.publish(self.twist)
        # rospy.sleep(sleep)
        # self.twist.linear.x = 1
        # self.pub.publish(self.twist)
        # rospy.sleep(sleep)
        # self.twist.linear.x = 1
        # self.pub.publish(self.twist)
        # rospy.sleep(sleep)
        # self.twist.linear.x = 1
        # self.pub.publish(self.twist)
        # rospy.sleep(sleep)
        # self.twist.linear.z = 0.2
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def left(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movement interrupted')
            return
        rospy.sleep(sleep)
        print('\n Moving left...')

        self.twist.linear.y = 0.6
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def right(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movement interrupted')
            return
        rospy.sleep(sleep)
        print('\n Moving right...')

        self.twist.linear.y = -0.6
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def backwards(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movement interrupted')
            return
        rospy.sleep(sleep)
        print('\n Going backwards...')

        self.twist.linear.x = -0.5
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def up(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movement interrupted')
            return
        rospy.sleep(sleep)
        print('\n Going up...')

        self.twist.linear.z = 1
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def down(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movement interrupted')
            return
        rospy.sleep(sleep)
        print('\n Going down...')

        self.twist.linear.z = -0.5
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def turn_left(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movement interrupted')
            return
        rospy.sleep(sleep)
        print('\n Turning left...')

        self.twist.angular.z = 0.5
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def turn_right(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movement interrupted')
            return
        rospy.sleep(sleep)
        print('\n Turning right...')
        
        self.twist.angular.z = -0.3
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def camera_pan(self, pan):
        print(f'\n Adjusting camera pan: {pan} degrees...')
        camera_twist = Twist()
        camera_twist.angular.z = pan
        self.pub_camera.publish(camera_twist)
        rospy.sleep(sleep)

    def camera_tilt(self, tilt):
        print(f'\n Adjusting camera tilt: {tilt} degrees...')
        camera_twist = Twist()
        camera_twist.angular.y = tilt
        rospy.sleep(sleep)
        self.pub_camera.publish(camera_twist)
        rospy.sleep(sleep)

    def reset_twist(self):
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
