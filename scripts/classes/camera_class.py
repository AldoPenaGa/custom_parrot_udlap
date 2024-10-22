import os
import cv2
from ultralytics import YOLO

# Obtain the current directory (where this file is located)
current_dir = os.path.dirname(os.path.abspath(__file__))

# The project root is one level up from the current directory
project_root = os.path.abspath(os.path.join(current_dir, '..'))

# Define the path to the YOLO model
model_path = os.path.join(project_root, 'models', 'best.pt')

class BebopCameraProcessor:
    def __init__(self):
        self.square = None
        self.biggest = 0
        self.center = (856 // 2, 480 // 2)
        self.model = YOLO(model_path)

    def process_image(self, cv_image):
        while not self.square:
            self.predict(cv_image)

        command = self.get_biggest_square(cv_image)
        self.square = None

        return cv_image, command

    def predict(self, cv_image):
        results = self.model.predict(source=cv_image)

        found_square = False
        for result in results:
            boxes = result.boxes
            if len(boxes) > 0:
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    width = x2 - x1
                    height = y2 - y1

                    if 0.9 <= (width / float(height)) <= 1.1:
                        area = width * height
                        if area > self.biggest:
                            self.biggest = area
                            self.square = [x1, y1, x2, y2]
                            found_square = True

        if not found_square:
            print("No valid square found, retrying...")

    def get_biggest_square(self, cv_image):
        command = None
        if self.square:
            x1, y1, x2, y2 = self.square
            cX = (x1 + x2) // 2
            cY = (y1 + y2) // 2

            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(cv_image, (cX, cY), 5, (0, 0, 255), -1)

            if cX < self.center[0] - 25:
                command = "a"
            elif cX > self.center[0] + 25:
                command = "d"
            else:
                command = "w"

        cv2.circle(cv_image, self.center, 2, (255, 0, 0), 2)

        return command
