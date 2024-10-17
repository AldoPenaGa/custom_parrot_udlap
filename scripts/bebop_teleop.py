#!/usr/bin/env python

import os
import sys
import rospy
import select
import termios
import threading
import tty
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist

# Current dir
current_dir = os.path.dirname(os.path.abspath(__file__))

# Project dir (up directly)
project_root = os.path.abspath(os.path.join(current_dir, '..'))

# Add to path project root dir path
sys.path.append(project_root)
from scripts.classes.bebop_movements import BebopMovements

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

# Clase principal que maneja tanto la teleoperación como los comandos automatizados
class BebopTeleop:
    def __init__(self):
        # Inicializa los publishers y subscribers
        self.pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('bebop/land', Empty, queue_size=10)
        self.pub_camera = rospy.Publisher('bebop/camera_control', Twist, queue_size=1)

        # Inicializa el nodo y la clase de movimientos
        rospy.init_node('teleop_control')
        self.movements = BebopMovements(self.pub, self.pub_takeoff, self.pub_land)

        # Suscribirse al tópico para comandos automatizados
        rospy.Subscriber('/bebop/command', String, self.command_callback)

        # Inicia el modo manual
        self.emergency_flag = [False]
        emergency_thread = threading.Thread(target=self.monitor_emergency_key)
        emergency_thread.daemon = True
        emergency_thread.start()

    def getKey(self):
        """
        Lee una tecla del teclado en modo manual.
        """
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def command_callback(self, msg):
        """
        Processes automated commands sent from another node.
        """
        command = msg.data
        rospy.loginfo(f"Received command: {command}")

        # Takeoff command
        if command == '1':  
            self.movements.initial_takeoff()
            rospy.loginfo("Taking off...")

        # Landing command
        elif command == '2':  
            self.movements.landing()
            rospy.loginfo("Landing...")

        # Command to ascend
        elif command == '+':  
            self.movements.twist.linear.z = 1
            self.pub.publish(self.movements.twist)
            rospy.sleep(1)
            self.movements.twist.linear.z = 0
            self.pub.publish(self.movements.twist)
            rospy.loginfo("Ascending...")

        # Command to descend
        elif command == '-':  
            self.movements.twist.linear.z = -1
            self.pub.publish(self.movements.twist)
            rospy.sleep(1)
            self.movements.twist.linear.z = 0
            self.pub.publish(self.movements.twist)
            rospy.loginfo("Descending...")

        # Process movement commands
        elif command in moveBindings:  
            x, y, z, th = moveBindings[command]
            self.movements.twist.linear.x = x
            self.movements.twist.linear.y = y
            self.movements.twist.linear.z = z
            self.movements.twist.angular.z = th
            self.pub.publish(self.movements.twist)
            rospy.loginfo(f"Moving drone with command: {command}")

        # Process camera movement commands
        elif command in cameraBindings:  
            pan, tilt = cameraBindings[command]
            camera_twist = Twist()
            camera_twist.angular.z = pan
            camera_twist.angular.y = tilt
            self.pub_camera.publish(camera_twist)
            rospy.loginfo(f"Moving camera with command: {command}")

    def teleop_mode(self):
        """
        Modo manual de teleoperación, activado por la tecla 't'.
        """
        print("\n--- Modo teleoperación activado ---")
        print("Aterriza presionando 2 o Ctrl+C para aterrizar y salir")
        
        while True:
            key = self.getKey()

            if key == '1':
                self.movements.initial_takeoff()
                print("\nDespegando...")
                break

            if key == '2': 
                self.movements.landing()
                print("\nAterrizando...")
                break

            if key in moveBindings:
                x, y, z, th = moveBindings[key]
                self.movements.twist.linear.x = x
                self.movements.twist.linear.y = y
                self.movements.twist.linear.z = z
                self.movements.twist.angular.z = th
                self.pub.publish(self.movements.twist)
            elif key in cameraBindings:
                pan, tilt = cameraBindings[key]
                camera_twist = Twist()
                camera_twist.angular.z = pan
                camera_twist.angular.y = tilt
                self.pub_camera.publish(camera_twist)
                print(f'\nControl de cámara: pan {pan} grados, tilt {tilt} grados')
            elif key == '\x03':  # Ctrl + C 
                print("\nAterrizando...")
                self.movements.landing()
                break

        self.movements.reset_twist()
        self.pub.publish(self.movements.twist)

    def monitor_emergency_key(self):
        """
        Monitorea teclas de emergencia como 't' para activar modo teleoperación.
        """
        while True:
            key = self.getKey()
            if key == 't':
                self.emergency_flag[0] = True
                self.teleop_mode()
            elif key == '\x03':  # Ctrl + C para aterrizar y salir
                print("\nAterrizando antes de salir...")
                self.movements.landing()
                rospy.signal_shutdown("\n Terminando por Ctrl-C")
                break

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    teleop = BebopTeleop()

    print(msg)

    try:
        # Mantener el nodo de ROS activo
        rospy.spin()

    finally:
        teleop.movements.reset_twist()
        teleop.pub.publish(teleop.movements.twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
