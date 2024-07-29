RETRIES = 6


class Status:
    """
    @class: Query ROBOTEQ controller for certain stats
    """
    def __init__(self, connection):
        # ROBOTEQ commands SDC2160 controller
        self.read_cmds = {
            "actual_velocity": "?SPE _",
            "battery_voltage": "?V 2_"
        }
        self.connect = connection

    def read_battery_12v(self) -> float:
        """
        Query battery voltage and parse the list of strings for battery voltage
        :return: Volts in float
        """
        batt_voltage = 0
        read_timer = 0.001 # seconds, anything faster would make battery voltage readings unpredictable

        for retry in range(RETRIES):
            resp = self.connect.handle_request(self.read_cmds["battery_voltage"], read_timer)
            resp = resp.split('\r')
            if len(resp) > 1 and len(resp[1]) > 1:
                break

        # Give me 132 in response list ['?V 2', 'V=132'] or ['V=132', '?V 2']
        for entry in resp:
            if "V=" in entry:
                batt_voltage = float(entry[2:]) / 10

        return float(batt_voltage)

    def read_velocity(self) -> tuple:
        """
        response is given as: SPE=3:3 or ?SPE=3:3 or any other variation

        :return: (3.0, 3.0) as a tuple for left and right side wheel velocities
        """
        read_timer = 0.1  # seconds, anything faster would make reading velocities inconsistent
        velocity_left_wheels = 0.0
        velocity_right_wheels = 0.0

        for retry in range(RETRIES):
            resp = self.connect.handle_request(self.read_cmds["actual_velocity"], read_timer)
            resp = "".join(c for c in resp if c.isdigit() or c == ' ' or c == '-')
            resp = resp.split(' ')
            if resp and len(resp) > 2:
                velocity_left_wheels = float(resp[1])
                velocity_right_wheels = float(resp[2])
                break

        return velocity_left_wheels, velocity_right_wheels
