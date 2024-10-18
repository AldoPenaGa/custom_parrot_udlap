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

    def initial_takeoff(self, mode_flag):

        if mode_flag not in ['automatic', 'teleop'] or rospy.is_shutdown():
            print('\n Estado inválido.')
            return
        rospy.sleep(1)

        print('\n Despegando...')
        self.pub_takeoff.publish(Empty())
        rospy.sleep(1)
        self.reset_twist()

    def landing(self, mode_flag):
        if mode_flag not in ['automatic', 'teleop'] or rospy.is_shutdown():
            print('\n Estado inválido')
            return
        self.reset_twist()
        print('\n Aterrizando...')

        rospy.sleep(sleep)
        self.pub.publish(self.twist)
        rospy.sleep(sleep)

        self.pub_land.publish(Empty())
        
        print('\n Aterrizaje hecho')
        self.reset_twist()

    def forward(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movimiento interrumpido.')
            return
        rospy.sleep(sleep)
        print('\n Avanzando...')

        self.twist.linear.x = 0.5
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def left(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movimiento interrumpido.')
            return
        rospy.sleep(sleep)
        print('\n Moviéndose a la izquierda...')

        self.twist.linear.y = 0.5
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def right(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movimiento interrumpido.')
            return
        rospy.sleep(sleep)
        print('\n Moviéndose a la derecha...')

        self.twist.linear.y = -0.5
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def backwards(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movimiento interrumpido.')
            return
        rospy.sleep(sleep)
        print('\n Retrocediendo...')

        self.twist.linear.x = -0.5
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def up(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movimiento interrumpido.')
            return
        rospy.sleep(sleep)
        print('\n Subiendo...')

        self.twist.linear.z = 0.5
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def down(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movimiento interrumpido.')
            return
        rospy.sleep(sleep)
        print('\n Bajando...')

        self.twist.linear.z = -0.5
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def turn_left(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movimiento interrumpido.')
            return
        rospy.sleep(sleep)
        print('\n Girando a la izquierda...')

        self.twist.angular.z = 0.5
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def turn_right(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movimiento interrumpido.')
            return
        rospy.sleep(sleep)
        print('\n Girando a la derecha...')
        
        self.twist.angular.z = -0.5
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def reset_twist(self):
        self.twist = Twist()
