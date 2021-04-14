#!/usr/bin/env python

"""
Node 'quaternionToEulerNode.py'

Transforms odometry orientation data (quaternion format) to euler angles
see https://www.theconstructsim.com/ros-qa-how-to-convert-quaternions-to-euler-angles/
"""

import rospy
import time
from sensor_msgs.msg import Imu
from turtlebot3burger.msg import EulerAngles
from tf.transformations import euler_from_quaternion, quaternion_from_euler

pubAngles = rospy.Publisher('euler_angles', EulerAngles, queue_size=10)
lastSendTime = 0

def quaternionToEulerNode():
    
    rospy.init_node('quaternionToEulerNode', anonymous=True)
    rospy.Subscriber("/imu", Imu, imuCallback)
    
    rospy.spin() #keeps script running until node terminated

def imuCallback(data):
    """ Callback for data coming from topic /imu
    
    :param data: data of type sensor_msgs/Imu
    """
    
    global lastSendTime
    
    if time.time() - lastSendTime > 0.25:
        lastSendTime = time.time()
        orientation_q = data.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        
        msg = EulerAngles()
        msg.roll = roll
        msg.pitch = pitch
        msg.yaw = yaw
        pubAngles.publish(msg)
    

if __name__ == '__main__':
    try:
        quaternionToEulerNode()
    except rospy.ROSInterruptException:
        pass
