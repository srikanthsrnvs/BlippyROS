import math
from math import floor


class Converter:

    def __init__(self, motor_type, gear_multiplier, **kwargs):
        assert isinstance(motor_type, str)
        assert (motor_type == "servo" or motor_type == "traction")
        assert isinstance(gear_multiplier, int)
        self.gear_multiplier = gear_multiplier
        self.type = motor_type
        if len(kwargs.items()) > 0:
            self.wheel_size = kwargs.get("wheel_size")

    def convert_angle_to_time_required(self, speed, angle):
        radius = math.sqrt((0.305**2) + (0.185**2))
        arc_length = angle/360 * 2 * radius
        time = arc_length/(speed*60*2*math.pi*self.wheel_size)
        return time

    def convert_steps_to_degrees(self, steps):
        assert self.type == "servo"
        percentage_of_steps = steps / 4096 / self.gear_multiplier
        degrees = percentage_of_steps * 360
        return int(floor(degrees))

    def convert_degrees_to_steps(self, degrees):
        assert self.type == "servo"
        percentage_of_movement = degrees / 360
        steps = percentage_of_movement * 4096 * self.gear_multiplier
        return int(floor(steps))

    def convert_speed_to_steps(self, speed):
        steps_per_second = speed / 60 * 256 * self.gear_multiplier
        return int(floor(steps_per_second))

    def convert_steps_to_speed(self, steps):
        rpm = 60 * steps / 256 / self.gear_multiplier
        return int(floor(rpm))

    def convert_acceleration_to_steps(self, acceleration):
        steps_per_second_squared = acceleration / 3.75 * self.gear_multiplier
        return int(floor(steps_per_second_squared))

    def convert_steps_to_acceleration(self, steps):
        rpm_per_second = steps * 3.75 / self.gear_multiplier
        return int(floor(rpm_per_second))

    def convert_meters_to_steps(self, meters):
        assert self.type == "traction"
        distance_per_rotation_of_wheel = self.wheel_size * math.pi * 2
        meters_as_rotation_multiplier = meters / distance_per_rotation_of_wheel
        steps = meters_as_rotation_multiplier * 4096 * self.gear_multiplier
        return int(floor(steps))

    def convert_steps_to_meters(self, steps):
        assert self.type == "traction"
        steps_as_multiplier = steps / 4096 / self.gear_multiplier
        distance = 2 * math.pi * self.wheel_size * steps_as_multiplier
        return int(floor(distance))
