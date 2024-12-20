#!/usr/bin/env python

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
    'i': (0, 5),    # Tilt up
    'k': (0, -5),   # Tilt down
    'j': (-5, 0),   # Pan left
    'l': (5, 0),    # Pan right
}

msg = """
---------------------------
   q    w   e       +: up
   a        d       -: down
   z    s   c
---------------------------
Take off: 1
Land: 2
---------------------------
Teleop mode: t
Automatic mode: y
---------------------------
Cam control:
---------------------------
   j    i    l       
        k    
---------------------------
CTRL-C to exit and land.
"""

class BebopTeleop:
    def __init__(self):
        rospy.init_node('teleop_control')

        self.pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('bebop/land', Empty, queue_size=10)
        self.pub_camera = rospy.Publisher('bebop/camera_control', Twist, queue_size=1)

        self.movements = BebopMovements(self.pub, self.pub_takeoff, self.pub_land, self.pub_camera)

        self.mode_flag = 'automatic'  # Begins on automatic mode

        rospy.Subscriber('/bebop/command_throttled', String, self.command_callback, queue_size=1)

        self.settings = termios.tcgetattr(sys.stdin)
        self.init_camera_position()


    def init_camera_position(self):
        self.movements.camera_tilt(-90)
        rospy.sleep(2)
        self.movements.initial_takeoff(self.mode_flag)
        rospy.sleep(2)
        self.movements.camera_tilt(10)
        rospy.sleep(2)

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

        if command == '2':
            self.movements.landing(self.mode_flag)
            rospy.loginfo("Landing...")

        elif command in command_to_method_mapping:
            method_name = command_to_method_mapping[command]
            movement_method = getattr(self.movements, method_name)
            rospy.loginfo(f"Moving: {method_name}")
            movement_method(self.mode_flag)

        elif command in cameraBindings:
            pan, tilt = cameraBindings[command]
            if pan != 0:
                self.movements.camera_pan(pan)
            if tilt != 0:
                self.movements.camera_tilt(tilt)
            rospy.loginfo(f"Moving cam: {command}")

        else:
            rospy.loginfo(f"Unknown command: {command}")

    def run(self):
        while not rospy.is_shutdown():
            key = self.getKey()

            if key == 't':
                self.mode_flag = 'teleop'
                print("\n--- TELEOP mode activated ---")
                print(msg)
                print("Press 'y' to enter automatic mode.")

            elif key == 'y':
                self.mode_flag = 'automatic'
                print("\n--- AUTOMATIC mode activated --- ")
                print("Press 't' to enter teleop mode.")
                self.movements.reset_twist()

            elif key == '\x03':  # Ctrl + C 
                print("\nLanding before exiting...")
                self.movements.landing(self.mode_flag)
                rospy.signal_shutdown("\nEnded by Ctrl-C")
                break

            elif self.mode_flag == 'teleop':
                if key == '1':
                    self.movements.initial_takeoff(self.mode_flag)
                    print("\nTaking off...")

                elif key == '2':
                    self.movements.landing(self.mode_flag)
                    print("\nLanding...")

                elif key in moveBindings:
                    x, y, z, th = moveBindings[key]
                    self.movements.twist.linear.x = x
                    self.movements.twist.linear.y = y
                    self.movements.twist.linear.z = z
                    self.movements.twist.angular.z = th
                    self.pub.publish(self.movements.twist)

                elif key in cameraBindings:
                    pan, tilt = cameraBindings[key]
                    if pan != 0:
                        self.movements.camera_pan(pan)
                    if tilt != 0:
                        self.movements.camera_tilt(tilt)
                    print(f'\nCamera control: pan {pan} degrees, tilt {tilt} degrees')

                else:
                    self.movements.reset_twist()
                    self.pub.publish(self.movements.twist)

            else:
                pass

if __name__ == "__main__":
    teleop = BebopTeleop()

    try:
        teleop.run()
    except rospy.ROSInterruptException:
        pass