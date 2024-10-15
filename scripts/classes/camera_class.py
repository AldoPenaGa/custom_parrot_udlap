import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
#import tensorflow as tf


class BebopCameraDisplay:


    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('bebop_camera_display', anonymous=True)

        # Initialize the CvBridge object
        self.bridge = CvBridge()

        # get the squares in frame
        self.square = None
        # get biggest area
        self.biggest = 0


        # center of camera
        self.center = (856//2, 480//2)

        # get nn classes
        self.classes = ['circle', 'kite', 'parallelogram', 'rectangle', 'rhombus', 'square', 'trapezoid', 'triangle']

        # Subscribe to the Bebop camera image topic
        self.image_sub = rospy.Subscriber("/bebop/image_raw", Image, self.image_callback)
        
        # Create a named window for displaying the video
        cv2.namedWindow("Bebop Camera", cv2.WINDOW_NORMAL)
        
        # Spin to keep the node running
        rospy.spin()


    def image_callback(self, msg):
        """
        Callback function to handle image messages from the drone's camera.
        """
        try:
            # Convert the ROS Image message to an OpenCV format
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # preprocess images
            #self.prepare_images()
            # get squares
            self.find_squares() 
            
            # Wait for a key press, with a short delay to allow OpenCV to refresh the window
            if cv2.waitKey(1) & 0xFF == ord('q'):
                # Exit if 'q' is pressed
                rospy.signal_shutdown("User exit")
        
        except Exception as e:
            rospy.logerr("Error converting or displaying image: %s", e)

    """
    def prepare_and_compute(self):
        "Prepares the frame to fit the neural network and 
        computes it through the neural network."
        # resize image
        self.cv_image = tf.image.resize(self.cv_image, [128, 128])
        # predict using model
        prediction = self.model.predict(self.cv_image)
    """


    def shutdown(self):
        """
        Properly shutdown the OpenCV window when the node is stopped.
        """
        cv2.destroyAllWindows()

    
    def find_squares(self):
        "Finds squares in an image"
        # apply gaussian blur to reduce noise
        self.blured = cv2.GaussianBlur(self.cv_image, (5,5), 0)
        # perform canny edge detection
        self.edges = cv2.Canny(self.blured, 50, 150)
        # find contours
        contours, _ = cv2.findContours(self.edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # Approximate the contour
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Check if the contour has 4 vertices
            if len(approx) == 4 and cv2.isContourConvex(approx):
                # Check the aspect ratio to determine if it's a square
                _, _, width, height = cv2.boundingRect(approx)
                aspect_ratio = float(width) / height

                # Allow imperfections in the aspect ratio
                if 0.9 <= aspect_ratio <= 1.1:
                    # Get the area to identify the largest square
                    area = cv2.contourArea(approx)
                    print(self.biggest)
                    # Check if the current square is the largest found
                    if area > self.biggest*.8:
                        self.biggest = area
                        self.square = approx
                        # Calculate the center of the self.square
                        M = cv2.moments(self.square)
                        if M["m00"] != 0:
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])
                        else:
                            cX, cY = 0, 0  # Avoid division by zero

                        # Draw a red point at the center of the square
                        cv2.circle(self.cv_image, (cX, cY), 5, (0, 0, 255), -1)  
                        cv2.drawContours(self.cv_image, [self.square], -1, (0, 255, 0), 2) 
        # draw center
        self.img = cv2.circle(self.cv_image, self.center, 2, (255, 0, 0), 2)
        # draw error circle
        self.img = cv2.circle(self.cv_image, self.center, 20, (255, 0, 0), 1) 

        # Show the result
        cv2.imshow("squares", self.cv_image)

    def find_squares_(self):
        """
        Finds squares in an image
        """
        return

        
           



if __name__ == '__main__':
    try:
        # Create and run the Bebop camera display node
        display = BebopCameraDisplay()
        
    except rospy.ROSInterruptException:
        pass