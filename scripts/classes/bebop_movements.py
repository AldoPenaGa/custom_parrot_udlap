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
        self.forward_count = 0  # Inicializar el contador de movimientos hacia adelante

    def initial_takeoff(self, mode_flag):

        if mode_flag not in ['automatic', 'teleop'] or rospy.is_shutdown():
            print('\n Estado inválido.')
            return
        rospy.sleep(1)

        print('\n Despegando...')
        self.pub_takeoff.publish(Empty())
        rospy.sleep(3)
        self.up(mode_flag)
        self.up(mode_flag)
        self.up(mode_flag)
        self.up(mode_flag)
        self.turn_right(mode_flag)
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

        self.twist.linear.x = 1
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.twist.linear.x = 1
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.twist.linear.x = 1
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.twist.linear.x = 1
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.twist.linear.x = 1
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.twist.linear.z = 0.2
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

        # Incrementar el contador de movimientos hacia adelante
        self.forward_count += 1
        print(f'\n Forward Count: {self.forward_count}')

        # Llamar a forward_window después de 3 movimientos hacia adelante
        if self.forward_count == 3:
            self.forward_window(mode_flag)
            self.forward_count = 0  # Resetear el contador después de llamar a forward_window


    def left(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movimiento interrumpido.')
            return
        rospy.sleep(sleep)
        print('\n Moviéndose a la izquierda...')

        self.twist.linear.y = 0.6
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def right(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movimiento interrumpido.')
            return
        rospy.sleep(sleep)
        print('\n Moviéndose a la derecha...')

        self.twist.linear.y = -0.6
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

        self.twist.linear.z = 1
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
        
        self.twist.angular.z = -0.3
        self.pub.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def forward_window(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movimiento interrumpido.')
            return
        rospy.sleep(sleep)
        print('\n Pasando ventana')
        self.forward(mode_flag)  # Llamar a forward tres veces más
        rospy.sleep(sleep)
        self.forward(mode_flag)
        rospy.sleep(sleep)
        self.forward(mode_flag)
        rospy.sleep(sleep)
        # Resetear el contador también aquí para asegurar que no continúe incrementando después de la ventana
        self.forward_count = 0

        

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
