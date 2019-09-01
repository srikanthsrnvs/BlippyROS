#!/usr/bin/env python3
import sys
sys.path.append('/home/parallels/rover_catkin_ws/src/rover_movement/src/Classes')
sys.path.append('/home/parallels/rover_catkin_ws/src/rover_movement/src/srv')

from diffrential_rover import DiffrentialRover
from traction import Traction
from pymodbus.client.sync import ModbusSerialClient
from rover_movement.srv import motor_command_server, motor_command_serverResponse
import rospy
from std_msgs.msg import String
import json
import uuid

command_queue = []
motor_status = []


def initialize_rover(modbus_port):
    global rover
    modbus_client = ModbusSerialClient(method='rtu', port=modbus_port, baudrate=57600, parity="E", timeout=1)
    modbus_client.connect()

    forward1 = Traction("bottomRight", 1, modbus_client, "right")
    forward2 = Traction("topLeft", 2, modbus_client, "left")
    forward3 = Traction("topRight", 3, modbus_client, "right")
    forward4 = Traction("bottomLeft", 4, modbus_client, "left")

    rover = DiffrentialRover([forward2, forward3], [forward4, forward1])
    rover.initialize()

def add_command_to_queue(req):
    global command_queue, motor_status
    data = json.loads(req.command)
    command_type = data["type"]
    command = data["command"]
    value = data["value"]
    identifier = uuid.uuid1()
    if command_type == "movement":
        command_queue.insert(0, (command, value, identifier))
        wait_for_command_to_execute(identifier)
        return motor_command_serverResponse("success")
    else:
        command_queue.append((command, value, identifier))
        wait_for_command_to_execute(identifier)
        stringified_json = json.dumps(motor_status)
        motor_status = []
        return motor_command_serverResponse(stringified_json)

def wait_for_command_to_execute(identifier):
    global command_queue
    while any(command[2] == identifier for command in command_queue):
        continue
    return
    
def run_command_queue():
    global command_queue
    while True:
        for element in command_queue:
            command, value, identifier = element
            rospy.loginfo("Running command: {},{}".format(command, value))
            execute_command((command, value))
            command_queue.remove(element)

def execute_command(raw_command):
    global command_queue, rover, motor_status
    command = raw_command[0]
    value = raw_command[1]

    if command == "steer_left":
        rover.steer_left(value)
        rospy.loginfo("Attempted to steer left..")
    elif command == "steer_right":
        rover.steer_right(value)
        rospy.loginfo("Attempted to steer right..")
    elif command == "go":
        rover.go(value, 10)
        rospy.loginfo("Attempted to go forwards..")
    elif command == "speed":
        rover.change_speed(value, 10)
        rospy.loginfo("Attempted to change speed..")
    elif command == "get_status":
        motor_status = rover.get_motor_status
        rospy.loginfo("Attempted to get status..")
    elif command == "stop":
        rover.stop()
        rospy.loginfo("Attempted to stop..")
    else:
        rospy.logerr("No such command!")

def start_listening_for_commands():
    rospy.init_node('motor_controller', anonymous=True)
    s = rospy.Service('motor_command_server', motor_command_server, add_command_to_queue)
    print("Ready to accept requests..")
    run_command_queue()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("No modbus port arguement specified")
    else:
        initialize_rover(sys.argv[1])
        start_listening_for_commands()

    