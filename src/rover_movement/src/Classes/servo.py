from pymodbus.client.sync import ModbusSerialClient
from conversions import Converter
from constants import *
from motor import Motor

Converter = Converter("servo", 1)


class Servo(Motor):

    def __init__(self, identifier, address, sync_client):
        assert isinstance(identifier, str)
        assert isinstance(sync_client, ModbusSerialClient)
        assert isinstance(address, int)
        Motor.__init__(self, identifier, address, sync_client)
        self.initialized = False

    def initialize(self):
        print("Attempting to initialize motor {}".format(self.identifier))
        command = self.sync_client.write_registers(TARGET_SELECT, [0, 1, 1], unit=self.address)
        if command.isError():
            self.initialized = False
        else:
            speed = Converter.convert_speed_to_steps(3000)
            self.initialized = True

    def get_current_angle(self):
        assert isinstance(self.sync_client, ModbusSerialClient)
        position = self.get_position()
        degrees = Converter.convert_steps_to_degrees(position)
        return degrees

    def set_angle(self, angle):
        assert isinstance(self.sync_client, ModbusSerialClient)
        assert isinstance(angle, int)
        position = Converter.convert_degrees_to_steps(angle)
        command = self.go_to_position(position)
        return command

    def get_steering_speed(self):
        command = self.sync_client.read_holding_registers(MOTOR_SPEED, 1, unit=self.address)
        speed = Converter.convert_steps_to_speed(command.registers[0])
        return speed

    def get_steering_torque(self):
        command = self.sync_client.read_holding_registers(MOTOR_TORQUE, 1, unit=self.address)
        torque = command.registers[0]
        return torque

    def set_angle_with_torque(self, torque, angle):
        assert isinstance(torque, int)
        assert isinstance(angle, int)
        command1 = self.sync_client.write_register(MOTOR_MAX_TORQUE, torque, unit=self.address)
        command2 = self.set_angle(angle)
        return command2
