#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class ThrottleCommandNode:
    def __init__(self):
        # Inicializa el nodo ROS
        rospy.init_node('throttle_command_node', anonymous=True)

        # Frecuencia a la que quieres throttlear (5 Hz)
        self.rate = rospy.Rate(0.5)  # Cambiado a 5 Hz

        # Suscribirse al tópico original de comandos
        self.sub = rospy.Subscriber("/bebop/command", String, self.command_callback)

        # Publicador para el nuevo tópico throttled
        self.pub = rospy.Publisher("/bebop/command_throttled", String, queue_size=1)

        # Almacenar el último comando recibido
        self.last_command = None

    def command_callback(self, msg):
        # Guardar el último comando recibido
        self.last_command = msg

    def start(self):
        # Ciclo principal: publica el último comando recibido a la frecuencia de 5 Hz
        while not rospy.is_shutdown():
            if self.last_command:
                self.pub.publish(self.last_command)  # Publicar el comando throttled
            self.rate.sleep()  # Mantener la frecuencia de 5 Hz

if __name__ == '__main__':
    try:
        throttle_command_node = ThrottleCommandNode()
        throttle_command_node.start()
    except rospy.ROSInterruptException:
        pass
