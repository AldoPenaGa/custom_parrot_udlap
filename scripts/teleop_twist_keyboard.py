#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Empty

import sys
import select
import termios
import tty
import time

#MENSAJE INICIAL
msg = """
Mapa de teclas:
---------------------------
   q    w   e
   a        d
   z    s   c
---------------------------
+ : sube (+z)
- : baja (-z)
---------------------------
Girar el dron (rot z),
utilizar Shift + A o Shift + D:

Izquierda: A
Derecha: D

Despegue: 1

Aterrizaje: 2

CTRL-C para salir.
"""

# MAPEO DE TECLAS

moveBindings = {
        # X, Y, Z, TH

    'w': (1, 0, 0, 0),          # Frente
    'a': (0, 1, 0, 0),          # Izquierda
    'd': (0, -1, 0, 0),         # Derecha
    's': (-1, 0, 0, 0),         # Atrás
    'q': (1, 1, 0, 0),          # Diagonal izquierda frente
    'e': (1, -1, 0, 0),         # Diagonal derecha frente
    'c': (-1, -1, 0, 0),        # Diagonal derecha atrás
    'z': (-1, 1, 0, 0),         # Diagonal izquierda atrás
    '+':(0,0,1,0),              # Sube
    '-':(0,0,-1,0),             # Baja
    'A':(0,0,0,1),              # Giro a la izquierda
    'D':(0,0,0,-1),             # Giro a la derecha
}

# FUNCIÓN OBTENCIÓN DE TECLA

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



def despegue_inicial():
    rospy.sleep(1)
    print('Despegando...')
    pub_takeoff.publish(empty)
    rospy.sleep(2)

    # Configuración estandar para ascenso
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0

    # Ascenso inicial:
    for i in range(3):
        twist.linear.z = 1
        rospy.sleep(0.2)
        pub.publish(twist)
        
    twist.linear.z = 0
    rospy.sleep(0.2)

def atterizaje():
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    print('Aterrizando...')
    rospy.sleep(0.1)
    pub.publish(twist)
    rospy.sleep(0.2)
    pub_land.publish(empty)
    
    print('Aterrizaje exitoso')

def frente():
    print('Pal frente')
    rospy.sleep(0.1)
    for i in range(2):
        twist.linear.x = 1
        rospy.sleep(0.2)
        pub.publish(twist)
    twist.linear.x = 0

def izquierda():
    print('Pa la izquierda')
    rospy.sleep(0.1)
    for i in range(1):
        twist.linear.y = 1
        rospy.sleep(0.4)
        pub.publish(twist)
        
    twist.linear.y = 0

def atras():
    print('Pal frente')
    rospy.sleep(0.1)
    for i in range(3):
        twist.linear.x = -1
        rospy.sleep(0.4)
        pub.publish(twist)

    twist.linear.x = 0

def derecha():
    print('Pa la derecha')
    rospy.sleep(0.1)
    for i in range(1):
        twist.linear.y = -1
        rospy.sleep(0.4)
        pub.publish(twist)
    twist.linear.y = 0
    rospy.sleep(0.2)

def abajo():
    print('Pa abajo')
    rospy.sleep(0.1)
    for i in range(2):
        twist.linear.z = -1
        rospy.sleep(0.4)
        pub.publish(twist)
    twist.linear.z = 0

def arriba():
    print('Pa abajo')
    rospy.sleep(0.1)
    for i in range(2):
        twist.linear.z = 1
        rospy.sleep(0.4)
        pub.publish(twist)
    twist.linear.z = 0
    

def giro_izq():
    print('Girando pa la izquierda...')
    rospy.sleep(0.1)
    for i in range (2):
        twist.angular.z = 1
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        rospy.sleep(0.2)
        pub.publish(twist)
        rospy.sleep(0.2)
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        rospy.sleep(0.2)
        pub.publish(twist)
    twist.angular.z = 0


# MAIN

if __name__ == "__main__":

    settings = termios.tcgetattr(sys.stdin)

    # PUBLISHER EN bebop/cmd_vel UTILIZANDO LOS VALORES DE TWIST
    pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
    # PUBLISHER DEL TAKEOFF
    pub_takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size=10)
    # PUBLISHER DEL LAND
    pub_land = rospy.Publisher('bebop/land', Empty, queue_size=10)
    
    # INICIALIZA EL NODO: teleop_twist_keyboard
    rospy.init_node('teleop_twist_keyboard')

    # OBTIENE PARÁMETROS ACTUALES
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)

    #rospy.set_param("~speed", 0.4)

    # DEFINE LOS VALORES INICIALES PARA X, Y Y Z.
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    # CREA EL OBJETO TWIST DE geometry_msgs/Twist.msg
    # TIENE 6 COMPONENTES:
    #  1 VECTOR DE 3 COMPONENTES LINEARES Y 
    #  1 VECTOR DE 3 COMPONENTES ANGULARES

# http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html

    twist = Twist()

    try:
        # Imprime el mensaje
        print (msg)

        # Declara el mensaje empty que literalmente es un mensaje vacío
        # http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Empty.html
        empty = Empty()
        
        while(1):
            # Asigna a key los valores del vector de la tecla presionada
            key = getKey()
            
            # Obtiene los valores de x, y, z y th del vector key
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]

            # Key = 1, despega.
            elif key == '1':
                print('Despegue normal')
                empty = Empty()
                pub_takeoff.publish(empty)
                
            # Key = 2, aterriza.
            elif key == '2':
                print('Aterrizaje normal')
                empty = Empty()
                pub_land.publish(empty)

            # Si no se presiona ninguno, x, y, z y theta serán defaulteados a 0.
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break
           
           # Se actualizan los valores de x, y y z para el twist.

            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th * turn
            pub.publish(twist)

    finally:
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)