#!/usr/bin/env python
import rospy
import cv2
import os
import sys
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Current dir
current_dir = os.path.dirname(os.path.abspath(__file__))

# Project dir (up directly)
project_root = os.path.abspath(os.path.join(current_dir, '..'))

# Add to path project root dir path
sys.path.append(project_root)

from scripts.classes.camera_class import BebopCameraProcessor




class BebopCameraNode:
    def __init__(self):
        # Inicializa el nodo ROS
        rospy.init_node('bebop_camera_node', anonymous=True)

        # Inicializa CvBridge para convertir imágenes de ROS a OpenCV
        self.bridge = CvBridge()

        # Publicador para comandos basados en la detección de cuadrados
        #self.command_pub = rospy.Publisher('/bebop/command', String, queue_size=10)

        # Suscripción al tópico de imágenes
        self.image_sub = rospy.Subscriber("/bebop/image_raw", Image, self.image_callback)

        # Instancia de la clase BebopCameraProcessor
        self.processor = BebopCameraProcessor()

        rospy.spin()

    def image_callback(self, msg):
        """
        Función callback para procesar la imagen de la cámara.
        """
        try:
            # Convertir la imagen de ROS a formato OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Procesar la imagen usando la clase
            processed_image, command = self.processor.process_image(cv_image)

            # Mostrar la imagen procesada
            cv2.imshow("Bebop Camera", processed_image)
            cv2.waitKey(1)

            # Si se detecta un comando (dirección), publicarlo
            if command:
                rospy.loginfo(f"Command: {command}")
                #self.command_pub.publish(command)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == '__main__':
    try:
        BebopCameraNode()
    except rospy.ROSInterruptException:
        pass
