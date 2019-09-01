#!/usr/bin/env python3
import sys
sys.path.append('/home/parallels/rover_catkin_ws/src/rover_movement/src/srv')

import pygame
import rospy
from std_msgs.msg import String
import json
from rover_movement.srv import *


rpm = 0
accelerate = 0
decelerate = 1
stop = 2

def initialize():
    pygame.init()
    pygame.joystick.init()
    _joystick = pygame.joystick.Joystick(4)
    _joystick.init()

def begin_listening_for_inputs():
    global rpm, accelerate, decelerate, stop
    rospy.init_node('master', anonymous=True)
    rospy.wait_for_service("motor_command_server")
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        command = {}
        for event in pygame.event.get():
            rospy.loginfo("{}".format(event))

            if event.type == pygame.JOYBUTTONDOWN:
                button = event.dict.get("button")
                if button == accelerate:
                    rpm += 5
                    command = {"command": "speed", "type": "movement", "value": rpm}
                elif button == decelerate:
                    rpm -= 5
                    command = {"command": "speed", "type": "movement", "value": rpm}
                elif button == stop:
                    rpm = 0
                    command = {"command": "stop", "type": "movement", "value": rpm}

            if event.type == pygame.JOYAXISMOTION:
                if event.axis == 0:  # this is the left stick
                    raw_dir = event.value
                    if -1 < raw_dir < 1:
                        if -1 < raw_dir < -0.1:
                            command = {"command": "steer_left", "type": "movement", "value": rpm}
                        elif 0.1 < raw_dir < 1:
                            command = {"command": "steer_right", "type": "movement", "value": rpm}
                        else:
                            command = {"command": "go", "type": "movement", "value": rpm}
        if not command == {}:
            try:
                srv = rospy.ServiceProxy('motor_command_server', motor_command_server)
                command_string = json.dumps(command)
                reply = srv(command_string)
                rospy.loginfo(reply.response)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        rate.sleep()


if __name__ == '__main__':
    initialize()
    try:
        begin_listening_for_inputs()
    except rospy.ROSInterruptException:
        pass