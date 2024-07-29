import serial
import time


RETRIES = 6
READ_SLEEP = 0.001  # 0.0001 seconds induces errors
RETRY_SLEEP = READ_SLEEP

class Connect:
    """
    @class: Connect to serial port and read and write commands
    """
    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = baudrate

    def handle_request(self, cmd: str, read_timer: float) -> str:
        """
        Write through serial port and read the entire line

        :param cmd: command to send
        :param read_timer: the time in which to sleep before reading serial port
        :return: string response from the serial port
        """
        resp = ""
        with serial.Serial(self.port, self.baudrate) as serial_handler:
            for retry in range(RETRIES):
                try:
                    serial_handler.write(cmd.encode())
                    time.sleep(read_timer)
                    resp = serial_handler.read_all().decode('utf-8')
                    if resp != "":
                        return resp
                except serial.serialutil.SerialException as e:
                    time.sleep(RETRY_SLEEP)
                    pass

        return resp

    def send_write_command(self, cmd: str) -> None:
        """
        :param cmd: command to send through the serial port
        :return: None
        """
        with serial.Serial(self.port, self.baudrate) as serial_handler:
            serial_handler.write(cmd.encode())
