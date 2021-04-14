#!/usr/bin/env python

"""
Node 'soundNode.py'

Implements functions on two buttons of OpenCR board for Turtlebot 3 Burger:
Button 1: Shutdown Raspberry Pi
Button 2: Play tones indicating last part of current IP address of the robot

In addition the node plays a start sound on starting and an end sound
before shutdown.

Caution: Press the buttons shortly to activate the functionalities,
pressing them longer causes the robot to do wheel-tests.
"""

import socket
import os

import rospy
from turtlebot3_msgs.msg import Sound, SensorState
from turtlebot3burger.msg import CustomSound

pubSound = rospy.Publisher('sound', Sound, queue_size=10)
pubCustom = rospy.Publisher('custom_sound', CustomSound, queue_size=10)
lastButtonState = 0
firstRun = True

def toneNode():
    global firstRun
    
    rospy.init_node('soundNode', anonymous=True)
    rospy.Subscriber("/sensor_state", SensorState, sensorStateCallback)
    
    hasIp = not not get_ip_address()
    while not rospy.is_shutdown():
        rospy.sleep(1)
        if not hasIp and firstRun and pubCustom.get_num_connections() > 0:
            firstRun = False
            sendCustomSound(440, 150)
            rospy.sleep(0.3)
            sendCustomSound(523, 150)
        
        if hasIp and firstRun and pubSound.get_num_connections() > 0:
            firstRun = False
            pubSound.publish(1) # start sound


def sensorStateCallback(data):
    """ Callback for data coming from topic /sensor_state
    
    :param data: data of type turtlebot3_msgs/SensorState
    """
    
    global lastButtonState
    
    if data.button == 0 and lastButtonState == 2:
        sendIpTone()
    if data.button == 0 and lastButtonState == 1:
        rospy.loginfo("shutting down robot...")
        pubSound.publish(0) # shutdown sound
        os.system('sudo poweroff')
    
    lastButtonState = data.button


def sendIpTone():
    """ Converts the last digits of the current IP number to ROS messages
    to topic /custom_sound
    """
    
    myip = get_ip_address()
    rospy.loginfo(myip)
    if myip == None:
        sendCustomSound(500, 70)
        rospy.sleep(0.1)
        sendCustomSound(1000, 70)
        rospy.sleep(0.1)
        sendCustomSound(500, 70)
        return
    
    lastpart = myip[myip.rindex('.') + 1:]
    rospy.loginfo(lastpart)
    for char in lastpart:
        count = int(char)
        if count == 0:
            sendCustomSound(250, 30)
            rospy.sleep(0.3)
        else:
            for num in range(count):
                sendCustomSound(500, 150)
                rospy.sleep(0.3)
        rospy.sleep(1)

def sendCustomSound(frequency, duration):
    """ Sends a ROS message to topic /custom_sound

    :param frequency: frequency of the sound to play
    :param duration: duration of the sound to play
    """
    
    msg = CustomSound(frequency = frequency, duration = duration)
    pubCustom.publish(msg)

def get_ip_address():
    """ Returns the current IP address.

    :return: the current IP address as String
    """
    
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except:
        return None
    


if __name__ == '__main__':
    try:
        toneNode()
    except rospy.ROSInterruptException:
        pass
