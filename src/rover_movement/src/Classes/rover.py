from dynamixel_servo import DynamixelServo
from traction import Traction


class Rover:

    def __init__(self, forward_motors, hind_motors, forward_steering, hind_steering):
        self.forward_motors = forward_motors
        self.hind_motors = hind_motors
        self.forward_steering = forward_steering
        self.hind_steering = hind_steering
        self.initialized = False
        assert isinstance(self.forward_steering[0], DynamixelServo) and isinstance(self.forward_steering[1], DynamixelServo) and isinstance(self.forward_motors[0], Traction) and isinstance(self.forward_motors[1], Traction) and isinstance(self.hind_steering[0], DynamixelServo) and isinstance(self.hind_steering[1], DynamixelServo) and isinstance(self.hind_motors[0], Traction))

    def initialize(self):
        for motor in self.forward_motors:
            motor.initialize()
        for motor in self.hind_motors:
            motor.initialize()
        for motor in self.forward_steering:
            motor.initialize()
            motor.steer(0)
        for motor in self.hind_steering:
            motor.initialize()
            motor.steer(0)
        self.initialized = True

    def go(self, speed, acceleration):
        for i in range(0, 2):
            motor1: Traction = self.forward_motors[i]
            motor2: Traction = self.hind_motors[i]
            motor1.go(speed, acceleration)
            motor2.go(speed, acceleration)

    def stop(self):
        for i in range(0, 2):
            motor1: Traction = self.forward_motors[i]
            motor2: Traction = self.hind_motors[i]
            motor1.stop_motor()
            motor2.stop_motor()

    def set_speed(self, speed, acceleration):
        for i in range(0, 2):
            motor1: Traction = self.forward_motors[i]
            motor2: Traction = self.hind_motors[i]
            motor1.set_acceleration_value(acceleration)
            motor2.set_acceleration_value(acceleration)
            motor1.change_current_speed(speed)
            motor2.change_current_speed(speed)

    def steer(self, angle):
        motor1: DynamixelServo = self.forward_steering[0]
        motor2: DynamixelServo = self.forward_steering[1]
        motor1.steer(angle)
        motor2.steer(angle)

    def get_motor_status(self):
        status = []
        for i in range(0, 2):
            status.append(self.front_motors[i].get_current_status_values())
            status.append(self.hind_motors[i].get_current_status_values())
        return status
