import rclpy
from rclpy.executors import MultiThreadedExecutor

# Custom submodules
from .connect import Connect
from .publishers import battery, drive_inverter
from .subscribers import move

PORT = "/dev/ttyACM0"
BAUDRATE = 115200


def main(args=None):
    connection = Connect(PORT, BAUDRATE)
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    executor.add_node(move.Move(connection))
    executor.add_node(battery.Publisher(connection))
    executor.add_node(drive_inverter.Publisher(connection))
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
