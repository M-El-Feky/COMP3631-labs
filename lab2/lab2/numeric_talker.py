import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8


class NumericTalker(Node):
    def __init__(self):
        super().__init__('talker')
        self.i = 0
        self.publisher = self.create_publisher(Int8, 'numeric_chatter', 10) # messeage setup
        self.timer = self.create_timer(0.5, self.talker_callback) # timing setup
        self.get_logger().info("Numeric talker started (publishing Int8 on /numeric_chatter)")

    def talker_callback(self):
        msg = Int8()
        msg.data = self.i
        self.publisher.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    talker = NumericTalker() # previous error here, always check naming
    rclpy.spin(talker)


if __name__ == '__main__':
    main()


