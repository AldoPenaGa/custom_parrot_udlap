#!/usr/bin/env python
import os
import cv2
import numpy as np

# Obtener el directorio actual (donde está este archivo)
current_dir = os.path.dirname(os.path.abspath(__file__))

# El directorio raíz del proyecto
project_root = os.path.abspath(os.path.join(current_dir, '..', 'classes'))

# Definir la ruta del modelo YOLO dentro de la carpeta 'models' ubicada en 'scripts/classes/'
model_path = os.path.join(project_root, 'models', 'best.pt')

from ultralytics import YOLO

class BebopCameraProcessor:
    def __init__(self):
        try:
            print(f"Cargando modelo YOLO desde: {model_path}")
            if not os.path.exists(model_path):
                raise FileNotFoundError(f"El modelo YOLO no se encontró en la ruta: {model_path}")
            self.model = YOLO(model_path)
        except Exception as e:
            print(f"Error cargando el modelo YOLO: {e}")
            raise e

    def process_image(self, cv_image):
        # Obtener el tamaño de la imagen y calcular el centro
        height, width = cv_image.shape[:2]
        self.center = (width // 2, height // 2)

        # Realizar la detección de cuadrados utilizando YOLO
        annotated_image, command_yolo = self.detect_with_yolo(cv_image.copy())

        # Realizar la detección de cuadrados utilizando OpenCV
        processed_image, command_opencv = self.detect_square(cv_image.copy())

        # Combinar las imágenes para mostrar ambas detecciones
        combined_image = cv2.addWeighted(annotated_image, 0.5, processed_image, 0.5, 0)

        # Decidir el comando basado en las detecciones (priorizar OpenCV si se detecta un cuadrado)
        if command_opencv:
            command = command_opencv
        else:
            command = command_yolo

        return combined_image, command

    def detect_with_yolo(self, cv_image):
        # Procesar la imagen con YOLO
        results = self.model.predict(source=cv_image)
        annotated_image = results[0].plot()

        # Inicializar variables
        biggest_area = 0
        square_coords = None
        command = None

        boxes = results[0].boxes  # Obtener las cajas detectadas
        if boxes is not None and boxes.xyxy is not None:
            # Convertir las coordenadas a numpy array
            xyxy = boxes.xyxy.cpu().numpy()  # (n, 4)
            for box_coords in xyxy:
                x1, y1, x2, y2 = box_coords
                width_box = x2 - x1
                height_box = y2 - y1
                if height_box == 0:
                    continue  # Evitar división por cero
                aspect_ratio = width_box / float(height_box)
                if 0.9 <= aspect_ratio <= 1.1:  # Verificar si es aproximadamente un cuadrado
                    area = width_box * height_box
                    if area > biggest_area:
                        biggest_area = area
                        square_coords = [int(x1), int(y1), int(x2), int(y2)]

        # Si se encontró un cuadrado, calcular el comando y dibujar sobre la imagen
        if square_coords:
            x1, y1, x2, y2 = square_coords
            cX = (x1 + x2) // 2
            cY = (y1 + y2) // 2
            cv2.circle(annotated_image, (cX, cY), 5, (0, 0, 255), -1)

            # Decidir el comando basado en la posición del cuadrado
            if cX < self.center[0] - 25:
                command = "a"  # Mover a la izquierda
            elif cX > self.center[0] + 25:
                command = "d"  # Mover a la derecha
            else:
                command = "w"  # Avanzar

            # Dibujar el cuadrado más grande
            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        else:
            print("No se encontró un cuadrado válido con YOLO.")

        # Dibujar el centro de la imagen
        cv2.circle(annotated_image, self.center, 5, (255, 0, 0), -1)

        return annotated_image, command

    def detect_square(self, cv_image):
        # Convertir la imagen de BGR a HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Definir los límites inferiores y superiores para el color azul en HSV
        lower_blue = np.array([110, 150, 150])
        upper_blue = np.array([130, 255, 255])

        # Crear una máscara que mantenga solo las regiones azules
        mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

        # Aplicar la máscara a la imagen original
        blue_only = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # Convertir a escala de grises y aplicar desenfoque Gaussian
        gray = cv2.cvtColor(blue_only, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Detectar bordes usando Canny
        edges = cv2.Canny(blurred, 50, 150)

        # Dilatar los bordes para hacerlos más gruesos
        kernel = np.ones((5, 5), np.uint8)
        thick_edges = cv2.dilate(edges, kernel, iterations=1)

        # Encontrar contornos en los bordes
        contours, _ = cv2.findContours(thick_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Inicializar variables
        biggest_area = 0
        square_contour = None
        command = None

        # Iterar sobre los contornos para encontrar cuadrados
        for contour in contours:
            # Aproximar el contorno a un polígono
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Verificar si el polígono tiene 4 lados y es convexo
            if len(approx) == 4 and cv2.isContourConvex(approx):
                area = cv2.contourArea(approx)
                if area > biggest_area:
                    biggest_area = area
                    square_contour = approx

        # Si se encontró un cuadrado
        if square_contour is not None:
            # Dibujar el contorno en la imagen
            cv2.drawContours(cv_image, [square_contour], -1, (0, 255, 0), 2)

            # Calcular el centro del cuadrado
            M = cv2.moments(square_contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                # Dibujar un círculo en el centro
                cv2.circle(cv_image, (cX, cY), 5, (0, 0, 255), -1)

                # Decidir el comando basado en la posición del cuadrado
                if cX < self.center[0] - 25:
                    command = "a"  # Mover a la izquierda
                elif cX > self.center[0] + 25:
                    command = "d"  # Mover a la derecha
                else:
                    command = "w"  # Avanzar
            else:
                print("No se pudo calcular el centro del cuadrado.")
        else:
            print("No se encontró un cuadrado válido con OpenCV.")

        # Dibujar el centro de la imagen
        cv2.circle(cv_image, self.center, 5, (255, 0, 0), -1)

        return cv_image, command
