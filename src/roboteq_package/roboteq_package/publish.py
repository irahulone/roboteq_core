import rclpy
from rclpy.node import Node


class Publisher(Node):
    def __init__(self, data, msg_type, topic: str, timer_period: float):
        super().__init__(topic + '_publisher')
        self.publisher = self.create_publisher(msg_type, topic, 10)
        self.data = data
        self.msg_type = msg_type
        self.get_logger().info('Initiating: Publisher "%s"' % data)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = self.msg_type()
        msg.data = self.data
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1



