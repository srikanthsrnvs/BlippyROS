from dynamixel_motor import DynamixelMotor


class DynamixelServo(DynamixelMotor):

    def __init__(self, port, identifier=254):
        DynamixelMotor.__init__(self, port, identifier)
        self.initialized = False
        self.calibration_factor = 0

    def initialize(self):
        self.enable_torque()
        self.turn_on_led()
        self.calibration_factor = self.get_position()
        self.initialized = True

    def steer(self, angle):
        position = angle/360 * 4095
        self.set_position(int(position))

    def get_status_values(self):
        temp = self.get_temperature()
        voltage = self.get_voltage()
        return {"temperature": temp, "voltage": voltage}