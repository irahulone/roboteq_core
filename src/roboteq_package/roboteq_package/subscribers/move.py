from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Int32MultiArray

QUEUESIZE = 10


class Move(Node):
    """
    @class: Any movement we would like to send
    """
    def __init__(self, connection):
        super().__init__("roboteq_move_subscriber")
        # ROBOTEQ commands SDC2160 controller
        self.move_cmds = {
            "move_wheels": "!M {} {}_"
        }
        self.connect = connection
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/mr1/ch_vals',
            self.move_motors_callback,
            QUEUESIZE,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

    def move_motors_callback(self, msg):
        cmd = self.move_cmds["move_wheels"].format(msg.data[0], msg.data[1])
        self.connect.send_write_command(cmd)

