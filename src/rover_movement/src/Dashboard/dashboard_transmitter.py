#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import socket
import json
from subprocess import call

connection: socket = None
addr = None
host = '127.0.0.1'
port = 1337


def setup_socket():
    global connection, host, port, addr
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        connection, addr = s.accept()
        with connection:
            print('Connected to BLE transmitter')
            start_listening_for_motor_data()

def send_data_to_dashboard(data):
    global connection
    rospy.loginfo(rospy.get_caller_id() + "Received motor status..")
    connection.sendall(data.data.encode("utf8"))

def start_listening_for_motor_data():
    rospy.init_node('dashboard_transmitter', anonymous=True)
    rospy.Subscriber("motor_data", String, send_data_to_dashboard)
    rospy.spin()

if __name__ == "__main__":
    setup_socket()