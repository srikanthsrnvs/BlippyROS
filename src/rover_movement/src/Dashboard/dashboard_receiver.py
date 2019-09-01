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
            print('Connected to BLE receiver')

def start_publishing_commands():
    global connection
    pub = rospy.Publisher('command_pipe', String, queue_size=10)
    rospy.init_node('dashboard_receiver', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    setup_socket()
    while not rospy.is_shutdown():
        data = connection.recv(1024)
        if data:
            pub.publish(data)
            rospy.loginfo("Published command: {}".format(data))
        rate.sleep()

if __name__ == "__main__":
    try:
        start_publishing_commands()
    except rospy.ROSInterruptException:
        pass
