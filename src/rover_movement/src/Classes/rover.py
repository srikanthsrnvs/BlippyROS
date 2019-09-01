from servo import Servo
from traction import Traction


class Rover:

    def __init__(self, forward_motors, hind_motors, forward_steering, hind_steering):
        self.forward_motors = forward_motors
        self.hind_motors = hind_motors
        self.forward_steering = forward_steering
        self.hind_steering = hind_steering
        self.initialized = False
        assert isinstance(self.forward_steering[0], Servo) and isinstance(self.forward_steering[1], Servo) and isinstance(self.forward_motors[0], Traction) and isinstance(self.forward_motors[1], Traction) and isinstance(self.hind_steering[0], Servo) and isinstance(self.hind_motors[0], Traction) and isinstance(self.hind_motors[1], Traction)

    def initialize(self):
        for motor in self.forward_motors:
            motor.initialize()
        for motor in self.hind_motors:
            motor.initialize()
        for motor in self.forward_steering:
            motor.initialize()
        for motor in self.hind_steering:
            motor.initialize()
        self.initialized = True

    def go(self, speed):
        for i in range(0, 2):
            motor1: Traction = self.forward_motors[i]
            motor2: Traction = self.hind_motors[i]
            motor1.go(speed, 10)
            motor2.go(speed, 10)

    def stop(self):
        for i in range(0, 2):
            motor1: Traction = self.forward_motors[i]
            motor2: Traction = self.hind_motors[i]
            motor1.stop_motor()
            motor2.stop_motor()

    def set_speed(self, speed, **kwargs):
        for i in range(0, 2):
            motor1: Traction = self.forward_motors[i]
            motor2: Traction = self.hind_motors[i]
            acceleration = kwargs.get("acceleration", 20)
            motor1.set_acceleration_value(acceleration)
            motor2.set_acceleration_value(acceleration)
            motor1.change_current_speed(speed)
            motor2.change_current_speed(speed)

    def steer_front(self, angle):
        for i in range(0, 2):
            motor1: Servo = self.forward_steering[i]
            motor1.set_angle(angle)

    # def steer_back(self, angle):
