#!/usr/bin/env python
import cv2

class BebopCameraProcessor:
    def __init__(self):
        # Initializing variables
        self.square = None
        self.biggest = 0
        self.center = (856 // 2, 480 // 2)  # Adjust according to camera resolution

    def process_image(self, cv_image):
        """
        Procesa la imagen para encontrar cuadrados y calcula su centro.
        Devuelve la imagen procesada con los cuadrados dibujados y el comando de dirección.
        """
        blured = cv2.GaussianBlur(cv_image, (5, 5), 0)

        edges = cv2.Canny(blured, 50, 150)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        command = None
        for contour in contours:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if len(approx) == 4 and cv2.isContourConvex(approx):

                _, _, width, height = cv2.boundingRect(approx)
                aspect_ratio = float(width) / height

                if 0.9 <= aspect_ratio <= 1.1:
                    area = cv2.contourArea(approx)

                    if area > self.biggest * 0.8:
                        self.biggest = area
                        self.square = approx

                        M = cv2.moments(self.square)
                        if M["m00"] != 0:
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])

                            # Determinar la dirección basado en el centro del cuadrado
                            if cX < self.center[0] - 50:
                                command = "a"
                            elif cX > self.center[0] + 50:
                                command = "d"
                            # elif cY < self.center[1] - 50:
                            #     command = "w"
                            # elif cY > self.center[1] + 50:
                            #     command = "s"

                        # Draw center and square
                        cv2.circle(cv_image, (cX, cY), 5, (0, 0, 255), -1)
                        cv2.drawContours(cv_image, [self.square], -1, (0, 255, 0), 2)

        # Draw camera center
        cv2.circle(cv_image, self.center, 2, (255, 0, 0), 2)
        return cv_image, command
