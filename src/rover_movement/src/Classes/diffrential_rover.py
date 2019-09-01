import math
from traction import Traction


class DiffrentialRover:

    def __init__(self, front_motors, hind_motors):
        self.front_motors = front_motors
        self.hind_motors = hind_motors

    def initialize(self):
        for motor in self.front_motors:
            motor.initialize()
        for motor in self.hind_motors:
            motor.initialize()

    def get_motor_status(self):
        status = []
        for i in range(0, 2):
            status.append(self.front_motors[i].get_current_status_values())
            status.append(self.hind_motors[i].get_current_status_values())
        return status

    def go(self, speed, acceleration):
        for i in range(0, 2):
            motor1: Traction = self.front_motors[i]
            motor2: Traction = self.hind_motors[i]
            motor1.go(speed, acceleration)
            motor2.go(speed, acceleration)

    def stop(self):
        for i in range(0, 2):
            motor1: Traction = self.front_motors[i]
            motor2: Traction = self.hind_motors[i]
            motor1.stop_motor()
            motor2.stop_motor()

    def slow_stop(self, deceleration):
        for i in range(0, 2):
            motor1: Traction = self.front_motors[i]
            motor2: Traction = self.hind_motors[i]
            motor1.change_current_speed(0, deceleration)
            motor2.change_current_speed(0, deceleration)

    def change_speed(self, speed, acceleration):
        for i in range(0, 2):
            motor1: Traction = self.front_motors[i]
            motor2: Traction = self.hind_motors[i]
            motor1.change_current_speed(speed, acceleration)
            motor2.change_current_speed(speed, acceleration)

    def steer_right(self, at_rpm):
        right_motors: [Traction] = [self.hind_motors[1], self.front_motors[1]]
        right_speed = -at_rpm
        for motor in right_motors:
            motor: Traction
            motor.change_current_speed(right_speed, 10)
        return right_speed

    def steer_left(self, at_rpm):
        left_motors: [Traction] = [self.hind_motors[0], self.front_motors[0]]
        left_speed = -at_rpm
        for motor in left_motors:
            motor: Traction
            motor.change_current_speed(-left_speed, 10)
        return left_speed


