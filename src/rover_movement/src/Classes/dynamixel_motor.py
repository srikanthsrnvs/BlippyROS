from dynamixel_sdk import *


class DynamixelMotor:

    def __init__(self, port, identifier):

        assert isinstance(port, str) and isinstance(identifier, int)

        self.port = PortHandler(port)
        self.identifier = identifier
        self.packetHandler = PacketHandler(1.0)

        if self.port.openPort():
            print("Opened the dynamixel servo port")
        else:
            print("Failed to open the dynamixel servo port")

        if self.port.setBaudRate(1000000):
            print("Initialized dynamixel servo")
        else:
            print("Failed to initialize dynamixel servo")

        self.packetHandler.write2ByteTxRx(self.port, self.identifier, 6, 4095)
        self.packetHandler.write2ByteTxRx(self.port, self.identifier, 8, 4095)

    def enable_torque(self):
        self.packetHandler.write1ByteTxRx(self.port, self.identifier, 24, 1)

    def disable_torque(self):
        self.packetHandler.write1ByteTxRx(self.port, self.identifier, 24, 0)

    def set_position(self, position):
        self.packetHandler.write2ByteTxRx(self.port, self.identifier, 30, position)

    def turn_on_led(self):
        self.packetHandler.write1ByteTxRx(self.port, self.identifier, 25, 1)

    def turn_off_led(self):
        self.packetHandler.write1ByteTxRx(self.port, self.identifier, 25, 0)

    def get_position(self):
        val, err, x = self.packetHandler.read2ByteTxRx(self.port, self.identifier, 36)
        if err != 0:
            print("Could not get position for motor {}".format(self.identifier))
            return err
        else:
            return val

    def get_voltage(self):
        val, err, x = self.packetHandler.read1ByteTxRx(self.port, self.identifier, 42)
        if err != 0:
            print("Could not get voltage for motor {}".format(self.identifier))
            return err
        else:
            return val

    def get_temperature(self):
        val, err, x = self.packetHandler.read1ByteTxRx(self.port, self.identifier, 43)
        if err != 0:
            print("Could not get temperature for motor {}".format(self.identifier))
            return err
        else:
            return val

    def set_id(self, set_id):
        val, err, x = self.packetHandler.write1ByteTxRx(self.port, self.identifier, 3, set_id)
        if err != 0:
            print("Could not set id for motor {}".format(self.identifier))
            return err
        else:
            return val


