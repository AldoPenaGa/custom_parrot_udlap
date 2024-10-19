#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

class ThrottleImageNode:
    def __init__(self):
        # Inicializa el nodo ROS
        rospy.init_node('throttle_image_node', anonymous=True)

        # Frecuencia a la que quieres throttlear (10 Hz)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Suscribirse al tópico original de imágenes
        self.sub = rospy.Subscriber("/bebop/image_raw", Image, self.image_callback)

        # Publicador para el nuevo tópico throttled
        self.pub = rospy.Publisher("/bebop/image_throttled", Image, queue_size=1)

        # Almacenar la última imagen recibida
        self.last_image = None

    def image_callback(self, msg):
        # Guardar la última imagen recibida
        self.last_image = msg

    def start(self):
        # Ciclo principal: publica la última imagen recibida a la frecuencia de 10 Hz
        while not rospy.is_shutdown():
            if self.last_image:
                self.pub.publish(self.last_image)  # Publicar la imagen throttled
            self.rate.sleep()  # Mantener la frecuencia de 10 Hz

if __name__ == '__main__':
    try:
        # Crear instancia de la clase y comenzar
        throttle_image_node = ThrottleImageNode()
        throttle_image_node.start()
    except rospy.ROSInterruptException:
        pass
