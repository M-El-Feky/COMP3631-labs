# Exercise 2 - detecting two colours, and filtering out the third colour and background.



import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal



class colourIdentifier(Node):
    def __init__(self):
        super().__init__('cI')
        self.sensitivity = 10
        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        # We covered which topic to subscribe to should you wish to receive image data
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning

    def callback(self, data):
    # Convert the received image into an opencv image (wrap in try/except in real use)
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridgeError: {e}')
            return

        # Convert the rgb image into a hsv image
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # ---- HSV bounds for TWO colours (GREEN + BLUE) ----
        # Green is around hue ~60
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])

        # Blue is around hue ~120
        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])

        # Filter out everything but particular colours using cv2.inRange()
        mask_green = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
        mask_blue  = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)

        # Combine the masks (two colours)
        mask_two_colours = cv2.bitwise_or(mask_green, mask_blue)

        # Apply the mask to the original image
        filtered = cv2.bitwise_and(image, image, mask=mask_two_colours)

        # Show results (debug-friendly)
        cv2.namedWindow('mask', cv2.WINDOW_NORMAL)
        cv2.imshow('mask', mask_two_colours)
        cv2.resizeWindow('mask', 320, 240)

        cv2.namedWindow('filtered', cv2.WINDOW_NORMAL)
        cv2.imshow('filtered', filtered)
        cv2.resizeWindow('filtered', 320, 240)

        cv2.waitKey(3)
    
    
def main():
    def signal_handler(sig, frame):
        rclpy.shutdown()

    
    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=None)
    cI = colourIdentifier()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(cI,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            continue
    except ROSInterruptException:
        pass
    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()
    
# Check if the node is executing in the main path
if __name__ == '__main__':
    main()
