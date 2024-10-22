import os
import cv2
from ultralytics import YOLO

class BebopCameraProcessor:
    def __init__(self):
        # Initialize the necessary variables
        self.square = None
        self.biggest = 0
        self.center = (856 // 2, 480 // 2)  # Adjust according to camera resolution
        # Load the YOLO trained model
        self.model = YOLO("/home/mikekun/catkin_ws/src/autonomous_drone_yolov8/autonomous_drone_yolov8/best.pt")

    def process_image(self, cv_image):
        """
        Processes the image to find squares using YOLO and calculates the center.
        Returns the processed image with the drawn squares and the direction command.
        """
        # Predict the image using YOLO, retrying until a square is detected
        while not self.square:
            self.predict(cv_image)

        # After prediction, calculate the largest square and its center
        command = self.get_biggest_square(cv_image)

        # Reset the square variable for the next image
        self.square = None

        # Return the processed image with the calculated command
        return cv_image, command

    def predict(self, cv_image):
        """
        Function to predict squares using YOLOv8.
        """

        # Define the path where you want to save the image from the drone
        temp_image_path = "/home/mikekun/catkin_ws/src/autonomous_drone_yolov8/autonomous_drone_yolov8/temp_image.jpg"

        # Save the OpenCV image (cv_image) to the specified path        
        cv2.imwrite(temp_image_path, cv_image)

        # Use YOLO to predict based on the saved image
        results = self.model.predict(source=temp_image_path)

        # Process YOLO results
        found_square = False
        for result in results:
            boxes = result.boxes
            if len(boxes) > 0:
                # Assuming YOLO detects squares
                for box in boxes:
                    # Get square coordinates
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    width = x2 - x1
                    height = y2 - y1

                    # Check if the shape is approximately square
                    if 0.9 <= (width / float(height)) <= 1.1:
                        area = width * height
                        if area > self.biggest:
                            self.biggest = area
                            # Store the largest square
                            self.square = [x1, y1, x2, y2]
                            found_square = True

        # After YOLO has processed the image, delete the temporary image file
        if os.path.exists(temp_image_path):
            os.remove(temp_image_path)

        # If no square was found, delete the image and continue
        if not found_square:
            print("No valid square found, retrying...")
            self.command_pub.publish("a") 

    def get_biggest_square(self, cv_image):
        """
        Function to get the largest square from predictions and calculate the direction.
        """
        command = None
        if self.square:
            # Get the center of the detected square
            x1, y1, x2, y2 = self.square
            cX = (x1 + x2) // 2
            cY = (y1 + y2) // 2

            # Draw the square on the image
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(cv_image, (cX, cY), 5, (0, 0, 255), -1)

            # Determine the direction based on the square's center
            if cX < self.center[0] - 25:
                command = "a"  # Move left
            elif cX > self.center[0] + 25:
                command = "d"  # Move right
            else:
                command = "w"  # Move forward
            

        # Draw the camera's center
        cv2.circle(cv_image, self.center, 2, (255, 0, 0), 2)

        return command
