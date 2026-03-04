import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal


class Robot(Node):
    def __init__(self):
        super().__init__('robot')

        # Publisher to move the robot (Lab 3)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz

        # Flags updated by camera callback
        self.green_detected = False
        self.blue_detected = False
        self.move_forwards = False
        self.move_backwards = False

        # Sensitivity for HSV hue window
        self.sensitivity = 10

        # Area thresholds (tune these)
        self.detect_area_min = 1500     # must be at least this big to count as "detected"
        self.too_close_area = 12000     # if green contour area > this -> back up
        self.too_far_area = 5000        # if green contour area < this -> move forward

        # Standard Twist messages
        self.twist_forward = Twist()
        self.twist_forward.linear.x = 0.12

        self.twist_backward = Twist()
        self.twist_backward.linear.x = -0.08

        self.twist_stop = Twist()  # all zeros

        # Image subscriber + bridge
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.callback, 10
        )

        # Windows (optional)
        cv2.namedWindow('camera_Feed', cv2.WINDOW_NORMAL)
        cv2.namedWindow('green_mask', cv2.WINDOW_NORMAL)
        cv2.namedWindow('blue_mask', cv2.WINDOW_NORMAL)

    def callback(self, data):
        # Convert ROS image -> OpenCV
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridgeError: {e}')
            return

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # --- GREEN bounds (Hue ~60) ---
        green_lower = np.array([60 - self.sensitivity, 100, 100])
        green_upper = np.array([60 + self.sensitivity, 255, 255])

        # --- BLUE bounds (Hue ~120) ---
        blue_lower = np.array([120 - self.sensitivity, 100, 100])
        blue_upper = np.array([120 + self.sensitivity, 255, 255])

        mask_green = cv2.inRange(hsv, green_lower, green_upper)
        mask_blue = cv2.inRange(hsv, blue_lower, blue_upper)

        # Optional clean-up to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)

        # Find contours
        green_contours, _ = cv2.findContours(mask_green, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        blue_contours, _ = cv2.findContours(mask_blue, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Reset flags each frame
        self.green_detected = False
        self.blue_detected = False
        self.move_forwards = False
        self.move_backwards = False

        # ---- BLUE: if detected, STOP immediately ----
        if len(blue_contours) > 0:
            cb = max(blue_contours, key=cv2.contourArea)
            blue_area = cv2.contourArea(cb)
            if blue_area > self.detect_area_min:
                self.blue_detected = True

        # ---- GREEN: follow if detected and blue not detected ----
        if (not self.blue_detected) and len(green_contours) > 0:
            cg = max(green_contours, key=cv2.contourArea)
            green_area = cv2.contourArea(cg)

            if green_area > self.detect_area_min:
                self.green_detected = True

                # Decide forward/back based on area (proxy for distance)
                if green_area > self.too_close_area:
                    self.move_backwards = True
                elif green_area < self.too_far_area:
                    self.move_forwards = True
                else:
                    # In the “good distance” band
                    pass

                # Draw green enclosing circle for debugging
                (x, y), radius = cv2.minEnclosingCircle(cg)
                cx, cy = int(x), int(y)
                cv2.circle(image, (cx, cy), int(radius), (0, 255, 0), 2)
                cv2.circle(image, (cx, cy), 3, (0, 255, 0), -1)

        # Show debug windows
        cv2.imshow('camera_Feed', image)
        cv2.resizeWindow('camera_Feed', 320, 240)
        cv2.imshow('green_mask', mask_green)
        cv2.resizeWindow('green_mask', 320, 240)
        cv2.imshow('blue_mask', mask_blue)
        cv2.resizeWindow('blue_mask', 320, 240)
        cv2.waitKey(3)

    # Movement helpers (publish once per call)
    def walk_forward(self):
        self.publisher.publish(self.twist_forward)

    def walk_backward(self):
        self.publisher.publish(self.twist_backward)

    def stop(self):
        self.publisher.publish(self.twist_stop)


def main():
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()

    rclpy.init(args=None)
    robot = Robot()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            # Priority: blue stops the robot
            if robot.blue_detected:
                robot.stop()

            # Otherwise follow green
            elif robot.green_detected:
                if robot.move_backwards:
                    robot.walk_backward()
                elif robot.move_forwards:
                    robot.walk_forward()
                else:
                    robot.stop()

            # If nothing detected, stop
            else:
                robot.stop()

            robot.rate.sleep()

    except ROSInterruptException:
        pass

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()