# Exercise 3 - If green object is detected, and above a certain size, then send a message (print or use lab2)

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
        self.green_detected = False
        self.sensitivity = 10
        self.area_threshold = 3000   # tune this (try 2000–6000)
        # Initialise any flags that signal a colour has been detected (default to false)

        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)

        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        # We covered which topic to subscribe to should you wish to receive image data
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning
        
        
    
    def callback(self, data):
        # Convert the received image into an opencv image
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridgeError: {e}')
            return

        # Convert the rgb image into a hsv image
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Set the upper and lower bounds for GREEN (Hue ~60)
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])

        # Filter out everything but green
        mask_green = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)

        # Optional: clean the mask a bit (reduces speckles)
        kernel = np.ones((5, 5), np.uint8)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask_green, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Default each frame
        self.green_detected = False

        if len(contours) > 0:
            # Biggest contour by area
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            # Only consider it if big enough
            if area > self.area_threshold:
                self.green_detected = True

                # Draw enclosing circle
                (x, y), radius = cv2.minEnclosingCircle(c)
                center_x, center_y = int(x), int(y)
                radius = int(radius)
                # Note the circle is being put on the image
                cv2.circle(image, (center_x, center_y), radius, (0, 255, 0), 2)
                cv2.circle(image, (center_x, center_y), 3, (0, 255, 0), -1)

                # Print / log once per frame while detected
                self.get_logger().info(f"GREEN detected! area={int(area)} center=({center_x},{center_y})")

        # Show windows (image + mask helps debugging)
        cv2.namedWindow('camera_Feed', cv2.WINDOW_NORMAL)
        cv2.imshow('camera_Feed', image)
        cv2.resizeWindow('camera_Feed', 320, 240)

        cv2.namedWindow('green_mask', cv2.WINDOW_NORMAL)
        cv2.imshow('green_mask', mask_green)
        cv2.resizeWindow('green_mask', 320, 240)

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
