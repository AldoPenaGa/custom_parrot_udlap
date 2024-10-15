#!/usr/bin/env python

import os
import sys

# Current dir
current_dir = os.path.dirname(os.path.abspath(__file__))

# Project dir (up directly)
project_root = os.path.abspath(os.path.join(current_dir, '..'))

# Add to path project root dir path
sys.path.append(project_root)

from scripts.classes.bebop_movements import BebopMovements

import rospy
import sys
import select
import termios
import threading
import tty
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

# Movements bindings
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

# Camera bindings
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
---------------------------
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
CTRL-C para salir (y aterrizar).
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def teleop_mode(movements, pub, pub_camera):
    print("\n--- Teleop mode activated ---")
    print("Aterriza presionando 2 o Ctrl+C para aterrizar y salir")
    
    while True:
        key = getKey()

        if key == '1':
            movements.initial_takeoff()
            print("\nTaking off...")
            break

        if key == '2': 
            movements.landing()
            print("\nAterrizando...")
            break

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
            print(f'\nControl de cámara: pan {pan} grados, tilt {tilt} grados')
        elif key == '\x03':  # Ctrl + C 
            print("\nAterrizando...")
            movements.landing()
            break

    movements.reset_twist()
    pub.publish(movements.twist)    

def monitor_emergency_key(movements, pub, pub_camera, emergency_flag):
    while True:
        key = getKey()
        if key == 't':
            # Teleop mode
            emergency_flag[0] = True
            teleop_mode(movements, pub, pub_camera)
        elif key == '\x03':  # Ctrl + C to land
            print("\nAterrizando antes de salir...")
            movements.landing()
            rospy.signal_shutdown("\n Terminando por Ctrl-C")
            break

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
        # Flag for teleop
        emergency_flag = [False]

        # Thread that monitors t key
        emergency_thread = threading.Thread(target=monitor_emergency_key, args=(movements, pub, pub_camera, emergency_flag))
        emergency_thread.daemon = True
        emergency_thread.start()

        print("\n Iniciando rutina: despegue, avance y aterrizaje...")
        
        ''''
        if not emergency_flag[0]:
            movements.initial_takeoff()
        if not emergency_flag[0]:
            movements.forward()
        if not emergency_flag[0]:
            movements.right()
        if not emergency_flag[0]:
            movements.left()
        if not emergency_flag[0]:
            movements.backwards()
        if not emergency_flag[0]:
            movements.turn_left()
        if not emergency_flag[0]:
            movements.turn_right()
        if not emergency_flag[0]:
            movements.landing()
        ""
        '''
        # Keep ros node running
        rospy.spin()

    finally:
        movements.reset_twist()
        pub.publish(movements.twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
