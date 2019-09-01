#!/usr/bin/env python3
import sys
sys.path.append('/home/parallels/rover_catkin_ws/src/rover_movement/src/Classes')
sys.path.append('/home/parallels/rover_catkin_ws/src/rover_movement/src/srv')

import rospy
from std_msgs.msg import String
from rover_movement.srv import *
import json

    
def get_motor_data():
    rospy.wait_for_service('motor_command_server')
    try:
        srv = rospy.ServiceProxy('motor_command_server', motor_command_server)
        command = {"type": "datacheck", "value": 0, "command": "get_status"}
        command_string = json.dumps(command)
        reply = srv(command_string)
        rospy.loginfo("Received response: {}".format(reply.response))
        return reply.response
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)

def start_publishing_motor_data():
    pub = rospy.Publisher('motor_data', String, queue_size=10)
    rospy.init_node('motor_data_hub', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        current_status = get_motor_data()
        pub.publish(current_status)
        rospy.loginfo("Published motor status..")
        rate.sleep()

if __name__ == '__main__':
    start_publishing_motor_data()