#!/usr/bin/env python

"""
Node 'simulationFeatureNode.py'

Node for replacing features of real Turtlebot3 Burger robot while running
gazebo simulation.

Listens for /custom_sound topic and creates the requested sounds.
Provides service to press Button 2 for sending IP address.
"""

import socket
import os

import rospy
from turtlebot3burger.msg import CustomSound
from turtlebot3_msgs.msg import SensorState
from std_srvs.srv import Empty, EmptyResponse

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/")
import util

pubSensorState = rospy.Publisher('sensor_state', SensorState, queue_size=10)

def simulationFeatureNode():
    global firstRun
    
    rospy.init_node('simulationFeatureNode', anonymous=True)
    rospy.Subscriber("/custom_sound", CustomSound, customSoundCallback)
    rospy.Service('pressButtonIP', Empty, handlePressButton)

    rospy.spin() #keeps script running until node terminated


def customSoundCallback(data):
    """ Callback for data coming from topic /custom_sound
    
    :param data: data of type turtlebot3burger/CustomSound
    """
    
    data.duration = util.limit(data.duration, 1, 10000)
    command = 'play -n synth %s sin %s' % (data.duration/1000.0, data.frequency)
    os.system(command)
    

def handlePressButton(req):
    """ Callback for service /pressButtonIP
    
    :param req: request data of type std_srvs/Empty
    """
    
    msg = SensorState()
    msg.button = 2
    pubSensorState.publish(msg)
    rospy.sleep(0.1)
    msg.button = 0
    pubSensorState.publish(msg)
    return EmptyResponse()


if __name__ == '__main__':
    try:
        simulationFeatureNode()
    except rospy.ROSInterruptException:
        pass
