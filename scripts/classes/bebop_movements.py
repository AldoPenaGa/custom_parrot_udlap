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
        self.pub_camera = pub_camera  # Nuevo publisher para controlar la cámara
        self.twist = Twist()

    def initial_takeoff(self, mode_flag):

        if mode_flag not in ['automatic', 'teleop'] or rospy.is_shutdown():
            print('\n Estado inválido.')
            return
        rospy.sleep(1)

        print('\n Despegando...')
        self.pub_takeoff.publish(Empty())
        rospy.sleep(1)
        self.up(mode_flag)
        rospy.sleep(1)
        self.up(mode_flag)
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

        self.twist.linear.x = 0.1
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def left(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movimiento interrumpido.')
            return
        rospy.sleep(sleep)
        print('\n Moviéndose a la izquierda...')

        self.twist.linear.y = 0.1
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def right(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movimiento interrumpido.')
            return
        rospy.sleep(sleep)
        print('\n Moviéndose a la derecha...')

        self.twist.linear.y = -0.1
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

    # Funciones nuevas para el control de la cámara
    def camera_pan(self, pan):
        print(f'\n Moviendo cámara pan: {pan} grados...')
        camera_twist = Twist()
        camera_twist.angular.z = pan
        self.pub_camera.publish(camera_twist)
        rospy.sleep(sleep)

    def camera_tilt(self, tilt):
        print(f'\n Moviendo cámara tilt: {tilt} grados...')
        camera_twist = Twist()
        camera_twist.angular.y = tilt
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
