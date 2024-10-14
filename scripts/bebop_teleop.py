#!/usr/bin/env python

import os
import sys

# Obtain current directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# The project root is one level up
project_root = os.path.abspath(os.path.join(current_dir, '..'))

# Add the project root so modules can be found
sys.path.append(project_root)

from scripts.classes.bebop_movements import BebopMovements

import rospy
import sys
import select
import termios
import tty
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

moveBindings = {
    'w': (1, 0, 0, 0),
    'a': (0, 1, 0, 0),
    'd': (0, -1, 0, 0),
    's': (-1, 0, 0, 0),
    'q': (0, 0, 0, 1),
    'e': (0, 0, 0, -1),
    'c': (-1, -1, 0, 0),
    'z': (-1, 1, 0, 0),
    '+': (0, 0, 1, 0),
    '-': (0, 0, -1, 0),
}

cameraBindings = {
    'i': (0, 0),
    'k': (0, -90),
    'j': (-90, 0),
    'l': (90, 0),
}

msg = """
---------------------------
   q    w   e       +: sube
   a        d       -: baja
   z    s   c
---------------------------
Despegue: 1
Aterrizaje: 2
Modo teleoperación: t
---------------------------
Girar dron (rot z):
. Izquierda: Shift + A
. Derecha: Shift + D
---------------------------
Control de la cámara:
---------------------------
   j    i    l       
        k    
---------------------------
CTRL-C para salir.
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def teleop_mode(movements, pub):
    print("Modo teleoperación activado.")
    while True:
        key = getKey()

        if key in moveBindings:
            x, y, z, th = moveBindings[key]
            movements.twist.linear.x = x
            movements.twist.linear.y = y
            movements.twist.linear.z = z
            movements.twist.angular.z = th
            pub.publish(movements.twist)
        elif key in cameraBindings:
            pan, tilt = cameraBindings[key]
            camera_twist = Twist()
            camera_twist.angular.z = pan
            camera_twist.angular.y = tilt
            pub_camera.publish(camera_twist)
            print(f'Control de cámara: pan {pan} grados, tilt {tilt} grados')
        elif key == '\x03':
            break
    movements.reset_twist()
    pub.publish(movements.twist)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
    pub_takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size=10)
    pub_land = rospy.Publisher('bebop/land', Empty, queue_size=10)
    pub_camera = rospy.Publisher('bebop/camera_control', Twist, queue_size=1)

    rospy.init_node('teleop_control')

    movements = BebopMovements(pub, pub_takeoff, pub_land)

    print(msg)

    try:
        # Ejecución automática de la rutina de despegue, avance y aterrizaje al iniciar el programa
        print("Iniciando rutina: despegue, avance y aterrizaje...")
        movements.initial_takeoff()
        movements.forward()
        movements.right()
        movements.left()
        movements.backwards()
        movements.turn_left()
        movements.turn_right()
        movements.landing()

        while True:
            key = getKey()

            if key == '1':
                movements.initial_takeoff()
            elif key == '2':
                movements.landing()
            elif key == 't':  # Activar modo teleoperación en caso de problemas
                teleop_mode(movements, pub)
            elif key == '\x03':
                break
    finally:
        movements.reset_twist()
        pub.publish(movements.twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)