#!/usr/bin/env python

# Libraries

import os
import sys
import rospy
import select
import termios
import tty
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist

# Import other classes
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(project_root)

from scripts.classes.bebop_movements import BebopMovements

# Mappings
moveBindings = {
    'w': (1, 0, 0, 0),
    'a': (0, 1, 0, 0),
    'd': (0, -1, 0, 0),
    's': (-1, 0, 0, 0),
    'q': (0, 0, 0, 1),
    'e': (0, 0, 0, -1),
    '+': (0, 0, 1, 0),
    '-': (0, 0, -1, 0),
}

cameraBindings = {
    'i': (0, 0),
    'k': (0, -90),
    'j': (-90, 0),
    'l': (90, 0),
}

# Teleop message
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
Modo automático: y
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

class BebopTeleop:
    def __init__(self):
        rospy.init_node('teleop_control')
    
        self.pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('bebop/land', Empty, queue_size=10)
        self.pub_camera = rospy.Publisher('bebop/camera_control', Twist, queue_size=1)

        self.movements = BebopMovements(self.pub, self.pub_takeoff, self.pub_land)

        self.mode_flag = 'automatic'  # Begins on automatic mode

        rospy.Subscriber('/bebop/command', String, self.command_callback)

        # Terminal configuration
        self.settings = termios.tcgetattr(sys.stdin)

    # Obtain keys
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        try:
            key = sys.stdin.read(1)
        except Exception:
            key = ''
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    # What to do when command arrives
    def command_callback(self, msg):
        if self.mode_flag != 'automatic':
            return

        command = msg.data
        rospy.loginfo(f"Comando recibido: {command}")

        command_to_method_mapping = {
            'w': 'forward',
            'a': 'left',
            'd': 'right',
            's': 'backwards',
            '+': 'up',
            '-': 'down',
            'q': 'turn_left',
            'e': 'turn_right',
        }

        # Landing command
        if command == '2':
            self.movements.landing(self.mode_flag)
            rospy.loginfo("Aterrizando...")

        # Movements command
        elif command in command_to_method_mapping:
            method_name = command_to_method_mapping[command]
            # method = getattr(self.movements, method_name)
            # method(self.mode_flag)
            rospy.loginfo(f"Ejecutando movimiento: {method_name}")

        # TODO: MODIDY COMMANDS TO MATCH CAMMERA BINDING
        elif command in cameraBindings:
            pan, tilt = cameraBindings[command]
            camera_twist = Twist()
            camera_twist.angular.z = pan
            camera_twist.angular.y = tilt
            self.pub_camera.publish(camera_twist)
            rospy.loginfo(f"Moviendo cámara con comando: {command}")

        else:
            rospy.loginfo(f"Comando desconocido: {command}")

    # HORSE MAIN FUNCTION
    def run(self):

        # TODO: SET CAMERA INITIALLY IN 0,0

        while not rospy.is_shutdown():
            key = self.getKey()

            if key == 't':
                self.mode_flag = 'teleop'
                print("\n--- Modo teleoperación activado ---")
                print(msg)
                print("Presiona 'y' para cambiar al modo automático.")

            elif key == 'y':
                self.mode_flag = 'automatic'
                print("\n--- Modo AUTOMÁTICO activado --- ")
                print("Presiona 't' para cambiar al modo teleop.")

            elif key == '\x03':  # Ctrl + C para aterrizar y salir
                print("\nAterrizando antes de salir...")
                self.movements.landing(self.mode_flag)
                rospy.signal_shutdown("\nTerminando por Ctrl-C")
                break

            elif self.mode_flag == 'teleop':
                # When teleop mode is activated
                if key == '1':
                    self.movements.initial_takeoff(self.mode_flag)
                    print("\nDespegando...")

                elif key == '2':
                    self.movements.landing(self.mode_flag)
                    print("\nAterrizando...")

                elif key in moveBindings:
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
                    print(f'\n Cammera control: pan {pan} degrees, tilt {tilt} degrees')

                else:
                    # If an incorrect key is pressed, it stops.
                    self.movements.reset_twist()
                    self.pub.publish(self.movements.twist)
            else:
                # Automatic mode, nothing is made except: 't', 'y', o Ctrl+C
                pass

if __name__ == "__main__":
    teleop = BebopTeleop()

    try:
        teleop.run()
    except rospy.ROSInterruptException:
        pass
