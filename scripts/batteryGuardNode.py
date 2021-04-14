#!/usr/bin/env python

"""
Node 'batteryGuardNode.py'

Monitors the battery state and shuts down the robot if battery voltage < 11V
"""

import socket
import os

import rospy
from turtlebot3_msgs.msg import Sound, SensorState

pubSound = rospy.Publisher('sound', Sound, queue_size=10)
shutdown = False

def batteryGuardNode():
    global firstRun
    
    rospy.init_node('batteryGuardNode', anonymous=True)
    rospy.Subscriber("/sensor_state", SensorState, sensorStateCallback)
    
    rospy.spin() #keeps script running until node terminated

def sensorStateCallback(data):
    """ Callback for data coming from topic /sensor_state
    
    :param data: data of type turtlebot3_msgs/SensorState
    """
    global shutdown

    if data.battery < 10.9 and not shutdown:
        shutdown = True
        rospy.loginfo("shutting down robot...")
        pubSound.publish(2) # battery alarm sound
        rospy.sleep(8)
        pubSound.publish(0) # shutdown sound
        os.system('sudo poweroff')

if __name__ == '__main__':
    try:
        batteryGuardNode()
    except rospy.ROSInterruptException:
        pass
