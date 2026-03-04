# Exercise 1 - Display an image of the camera feed to the screen

#from __future__ import division
import threading
import sys, time
import cv2 # imports computer vision
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

        # Bridge converts ROS Image messages <-> OpenCV images
        self.bridge = CvBridge()

        # Subscribe to the TurtleBot3 camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10
        )

        # Optional: create the window once
        cv2.namedWindow('camera_Feed', cv2.WINDOW_NORMAL)

    def callback(self, data):
        self.get_logger().info("Image received")
        # Convert the received image into an OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridgeError: {e}')
            return
        '''
        # Show the image
        cv2.imshow('camera_Feed', cv_image)
        cv2.resizeWindow('camera_Feed', 320, 240)
        cv2.waitKey(3)
        '''
        # 1) Convert BGR -> HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 2) Green range in HSV (OpenCV Hue is 0..179)
        lower_green = np.array([50, 100, 100])
        upper_green = np.array([80, 255, 255])

        # 3) Mask: white where green pixels are, black elsewhere
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # 4) Apply mask to original image
        green_only = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # 5) Show both (helps you debug)
        cv2.imshow('mask', mask)
        cv2.imshow('green_only', green_only)
        cv2.waitKey(3)
        

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
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
