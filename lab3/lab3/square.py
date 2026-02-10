import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.exceptions import ROSInterruptException
import signal


class SquareWalker(Node):
    def __init__(self):
        super().__init__('squarewalker')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz

    def drive_forward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.2   # forward
        desired_velocity.angular.z = 0.0  # no turning
        for _ in range(30):               # 3 seconds at 10 Hz
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def turn_left_90(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0
        desired_velocity.angular.z = 0.5  # turn in place

        # 90deg = pi/2 rad. Time = (pi/2) / 0.5 ≈ 3.14 s
        # 3.14 s at 10 Hz ≈ 31 ticks
        for _ in range(31):
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def stop(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0
        desired_velocity.angular.z = 0.0
        self.publisher.publish(desired_velocity)


def main():
    def signal_handler(sig, frame):
        square_walker.stop()
        rclpy.shutdown()

    rclpy.init(args=None)
    square_walker = SquareWalker()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(square_walker,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            for _ in range(4):
                square_walker.drive_forward()
                square_walker.turn_left_90()
    except ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

