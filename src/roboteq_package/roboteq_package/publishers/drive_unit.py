from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# Custom submodules
from roboteq_package.status import Status


class Publisher(Node):
    """
    @class: Publish any stats regarding the drive inverter
    """
    def __init__(self, connection):
        super().__init__('roboteq_drive_inverter_pub')
        self.get_stat = Status(connection)
        self.publisher = self.create_publisher(Float32MultiArray, "mr1/du_velocity", 10)
        di_velocity_period = 0.5  # seconds
        self.timer = self.create_timer(di_velocity_period,
                                       self.di_velocity_callback,
                                       callback_group=MutuallyExclusiveCallbackGroup())
        self.i = 0

    def di_velocity_callback(self):
        """
        Get velocity of wheels and publish
        :return:
        """
        msg = Float32MultiArray()
        response = self.get_stat.read_velocity()
        msg.data = [float(value) for value in response]

        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1



