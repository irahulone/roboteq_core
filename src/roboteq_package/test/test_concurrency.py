import time
import threading
import serial

PORT = "/dev/ttyACM0"
BAUDRATE = 115200
RETRIES = 6
READ_SLEEP = 0.001  # 0.0001 seconds induces errors



class Status:
    def __init__(self, connection):
        # ROBOTEQ commands SDC2160 controller
        self.read_cmds = {
            "actual_velocity": "?SPE _",
            "battery_voltage": "?V 2_"
        }
        self.connect = connection

    def read_battery_12v(self) -> None:
        """
        Query battery voltage and parse the list of strings for battery voltage
        :return:
        """
        batt_voltage = 0

        for retry in range(50):
            self.connect.handle_request(self.read_cmds["battery_voltage"])
            resp = resp.split('\r')
            print(f"battery response: {resp}")

    def read_velocity(self):
        """
        response is given as: SPE=3:3 or ?SPE=3:3 or any other variation

        :return: (3.0, 3.0) as a tuple for left and right side wheel velocities
        """
        velocity_left_wheels = 0.0
        velocity_right_wheels = 0.0

        for retry in range(50):
            resp = self.connect.handle_request(self.read_cmds["actual_velocity"])
            print(f"Velocity response: {resp}")


class Connect:
    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = baudrate

    def handle_request(self, cmd: str) -> str:
        resp = ""
        with serial.Serial(self.port, self.baudrate) as ser:
            for retry in range(RETRIES):
                ser.write(cmd.encode())
                time.sleep(READ_SLEEP)
                resp = ser.read_all().decode('utf-8')
                if resp != "":
                    return resp

        return resp

    def send_write_command(self, cmd: str) -> None:
        with serial.Serial(self.port, self.baudrate) as ser:
            ser.write(cmd.encode())


def move(connect):
    move_cmds = {
        "move_wheels": "!M {} {}_"
    }
    for i in range(50):
        connect.send_write_command(move_cmds["move_wheels"].format("5", "5"))


def main():
    connect = Connect(PORT, BAUDRATE)
    status = Status(connect)
    t1 = threading.Thread(target=move(connect))
    t2 = threading.Thread(target=status.read_velocity())
    t3 = threading.Thread(target=status.read_battery_12v())

    t1.start()
    t2.start()
    t3.start()


if __name__ == '__main__':
    main()