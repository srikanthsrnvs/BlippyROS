from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadBuilder
from conversions import Converter
from constants import *
from motor import Motor

Converter = Converter("traction", 16, wheel_size=0.1)


class Traction(Motor):
    def __init__(self, identifier, address, sync_client, type):
        Motor.__init__(self, identifier, address, sync_client)
        self.initialized = False
        if type == "left":
            self.multiplier = -1
        elif type == "right":
            self.multiplier = 1
        else:
            self.multiplier = 1
        self.current_speed = 0

    def initialize(self):
        print("Attempting to initialize {}".format(self.identifier))
        builder = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
        builder.add_16bit_int(0)
        builder.add_16bit_int(self.multiplier)
        builder.add_16bit_int(1)
        command = self.sync_client.write_registers(TARGET_SELECT, builder.to_registers(), unit=self.address)
        if command.isError():
            print("Could not initialize {}. Reason: {}".format(self.identifier, command))
            self.initialized = False
        else:
            print("Initialized {}".format(self.identifier))
            self.initialized = True

    def change_current_speed(self, to_speed, with_acc):
        assert self.initialized
        set_acc = self.set_acceleration_value(with_acc)
        set_dec = self.set_acceleration_value(with_acc)
        if set_acc and set_dec:
            steps = Converter.convert_speed_to_steps(to_speed)
            mode = self.get_mode()
            if mode == 21:
                return self.set_max_speed(steps)
            elif mode == 33:
                return self.set_target(steps)
            else:
                return False
        else:
            return False

    def set_acceleration_value(self, acceleration):
        assert self.initialized
        steps = Converter.convert_acceleration_to_steps(acceleration)
        command = self.set_max_acceleration(steps)
        return command

    def go(self, speed, acceleration):
        assert self.initialized
        speed_units = Converter.convert_speed_to_steps(speed)
        acc_units = Converter.convert_acceleration_to_steps(acceleration)
        return self.go_with_speed(speed_units, acc_units)

    def go_to(self, distance, speed, acceleration):
        assert self.initialized
        speed_units = Converter.convert_speed_to_steps(speed)
        acc_units = Converter.convert_acceleration_to_steps(acceleration)
        distance_units = Converter.convert_meters_to_steps(distance)
        return self.go_to_position(distance_units, speed_units, acc_units)

    def change_max_speed(self, speed):
        assert self.initialized
        speed_units = Converter.convert_speed_to_steps(speed)
        self.set_max_speed(speed_units)

    def get_current_status_values(self):
        status = self.get_current_status()
        formatted_status = {"position": status.get("position"),
                            "speed": Converter.convert_steps_to_speed(status.get("speed")),
                            "torque": status.get("torque") / 1000,
                            "voltage": status.get("voltage") / 32767 * 28,
                            "current": status.get("current") / 32767 * 25,
                            "electronics_temp": status.get("electronics_temp") / 100,
                            "motor_temp": status.get("motor_temp") / 100,
                            "error": status.get("error")}
        return formatted_status

    def get_max_speed_value(self):
        steps = self.get_max_speed()
        speed = Converter.convert_steps_to_speed(steps)
        return speed

    def get_max_acceleration_value(self):
        steps = self.get_max_acceleration()
        acc = Converter.convert_steps_to_acceleration(steps)
        return acc

    def get_max_torque_value(self):
        torque = self.get_max_torque()
        return torque

    def get_current_speed_values(self):
        steps = self.get_current_target()
        speed = Converter.convert_steps_to_speed(steps)
        return speed
