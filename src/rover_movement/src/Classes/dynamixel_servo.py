from dynamixel_motor import DynamixelMotor


class DynamixelServo(DynamixelMotor):

    def __init__(self, port, identifier=254):
        DynamixelMotor.__init__(self, port, identifier)
        self.initialized = False

    def initialize(self):
        self.enable_torque()
        self.turn_on_led()
        self.initialized = True

    def steer(self, angle):
        position = angle/360 * 4095
        self.set_position(int(position))

    def get_status_values(self):
        temp = self.get_temperature()
        voltage = self.get_voltage()
        return {"temperature": temp, "voltage": voltage}