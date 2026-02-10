import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.exceptions import ROSInterruptException
import signal


class CircleWalker(Node):
    def __init__(self):
        super().__init__('circlewalker')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz

    def drive_circle(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.2   # Forward 0.2 m/s
        desired_velocity.angular.z = 0.2  # Turn left 0.2 rad/s

        # Publish for 3 seconds (30 ticks at 10 Hz), same structure as your script
        for _ in range(30):
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def stop(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0
        desired_velocity.angular.z = 0.0
        self.publisher.publish(desired_velocity)


def main():
    def signal_handler(sig, frame):
        circle_walker.stop()
        rclpy.shutdown()

    rclpy.init(args=None)
    circle_walker = CircleWalker()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(circle_walker,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            circle_walker.drive_circle()
    except ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
