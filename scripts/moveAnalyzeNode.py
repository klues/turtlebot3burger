#!/usr/bin/env python

"""
Node 'moveAnalyzeNode.py'

Analyzes topics /imu, /scanAnalyzed, /cmd_vel and calculates approximate
movement and collision values which are published on topic /movement
"""

import rospy
import math
import time
from numpy import mean
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from turtlebot3burger.msg import ScanAnalyzed
from turtlebot3burger.msg import Movement

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/")
import util

pubMovement = rospy.Publisher('movement', Movement, queue_size=10)

# wheel movement
velForward = False
velBackward = False
velMovingLinear = False
velMovingAngular = False
velLinear = 0
velAngular = 0
velStartTime = 0

# scan movement
lastMinBack = None
lastMinFront = None
scanForward = False
scanBackward = False

# imu movement
movingImu = False

# collision
collisionAvg = 0

def moveAnalyzeNode():
    
    global collisionAvg
    
    rospy.init_node('moveAnalyzeNode', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, velCallback)
    rospy.Subscriber("/imu", Imu, imuCallback)
    rospy.Subscriber("/scanAnalyzed", ScanAnalyzed, scanCallback)
    
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        msg = Movement()
        msg.moving_backward_wheels = velBackward
        msg.moving_forward_wheels = velForward
        msg.moving_backward_scan = scanBackward
        msg.moving_forward_scan = scanForward
        msg.moving_scan = msg.moving_backward_scan or msg.moving_forward_scan
        msg.moving_imu = movingImu
        msg.moving_backward = movingImu and scanBackward and velBackward
        msg.moving_forward = movingImu and scanForward and velForward
        msg.moving = msg.moving_backward or msg.moving_forward
        msg.collision_forward = velForward and (not msg.moving_forward)
        msg.collision_backward = velBackward and (not msg.moving_backward)
        msg.collision = msg.collision_forward or msg.collision_backward
        msg.collision_avg = collisionAvg
        pubMovement.publish(msg)
        
        colInt = 1 if msg.collision else 0
        collisionAvg = util.movingAvg(colInt, "collisionAvg", 10)
        

def scanCallback(data):
    """ Callback for data coming from topic /scanAnalyzed
    
    :param data: data of type turtlebot3burger/ScanAnalyzed
    """
    global lastMinFront, lastMinBack, scanForward, scanBackward
    
    # limit ranges to 0.5m since bigger values are inaccurate and
    # movement approx doesn't work well with them. See datasheet LIDAR HLS-LFCD2
    range_front = util.limit(data.range_front_min, 0, 0.5)
    range_back = util.limit(data.range_back_min, 0, 0.5)
    limit_front = range_front == 0.5
    limit_back = range_back == 0.5
    
    # calculating moving sums of diffs of range values
    # -> should be near 0 if robot isn't moving
    lastMinFront = lastMinFront or range_front
    lastMinBack = lastMinBack or range_back
    avgCount = 10
    movingFrontSum = util.movingSum(range_front - lastMinFront, "movingFrontSum", avgCount)
    movingBackSum = util.movingSum(range_back - lastMinBack, "movingBackSum", avgCount)
    
    # calc movement values based on moving sums
    thresholdFactor = 10
    threshold = thresholdFactor * 0.01 / avgCount
    scanForwardB = movingBackSum > threshold
    scanForwardF = movingFrontSum < threshold * (-1)
    scanBackwardB = movingBackSum < threshold * (-1)
    scanBackwardF = movingFrontSum > threshold
    
    scanReliable = not (limit_back and limit_front)
    lidarRoundTime = (60.0 / 300) # 0.2s, see datasheet
    if (not scanReliable) or (time.time() - velStartTime) < (lidarRoundTime * avgCount):
        scanBackward = velBackward
        scanForward = velForward
    elif limit_back:
        scanBackward = scanBackwardF
        scanForward = scanForwardF
    elif limit_front:
        scanBackward = scanBackwardB
        scanForward = scanForwardB
    else:
        scanBackward = scanBackwardB or scanBackwardF
        scanForward = scanForwardB or scanForwardF
    
    lastMinFront = range_front
    lastMinBack = range_back
    
    #~ print "back: " + str(range_back)
    #~ print "front: " + str(range_front)
    #~ print "front min: " + str(data.range_front_min)
    #~ print "back min: " + str(data.range_back_min)
    #~ print "movingFrontSum: " + str(movingFrontSum)
    #~ print "movingBackSum: " + str(movingBackSum)
    #~ print "scanForwardB: " + str(scanForwardB)
    #~ print "scanForwardF: " + str(scanForwardF)
    #~ print "scanBackwardB: " + str(scanBackwardB)
    #~ print "scanBackwardF: " + str(scanBackwardF)
    
    
def velCallback(data):
    """ Callback for data coming from topic /cmd_vel
    
    :param data: data of type geometry_msgs/Twist
    """
    global velForward, velBackward, velMovingLinear, velMovingAngular
    global velLinear, velAngular, velStartTime, scanForward, scanBackward, movingImu
    
    velLinear = data.linear.x
    velAngular = data.angular.z
    velForward = velLinear > 0
    velBackward = velLinear < 0
    
    if not velMovingLinear and abs(data.linear.x) > 0:
        velStartTime = time.time()
        
        #init scan + imu movement values since real scan values have a delay 
        scanBackward = velBackward
        scanForward = velForward
        movingImu = velForward or velBackward
    
    velMovingLinear = velForward or velBackward
    velMovingAngular = abs(velAngular) > 0
    
    #~ print "velForward" + str(velForward)
    #~ print "velBackward" + str(velBackward)
    #~ print "velMovingLinear" + str(velMovingLinear)
    #~ print "velMovingAngular" + str(velMovingAngular)

def imuCallback(data):
    """ Callback for data coming from topic /scan
    
    :param data: data of type turtlebot3burger/ScanAnalyzed
    """
    global movingImu
    
    # moving variance of linear acceleration is bigger if robot is moving
    threshold = util.mapRange(abs(velLinear), 0.01, 0.1, 0.002, 0.01)
    threshold = util.limit(threshold, 0.002, 0.01)
    accAvgX = util.movingVar(data.linear_acceleration.x, "linAccX", 50)
    movingImu = accAvgX > threshold
    
    if (time.time() - velStartTime) < 1:
        movingImu = velMovingLinear
    

if __name__ == '__main__':
    try:
        moveAnalyzeNode()
    except rospy.ROSInterruptException:
        pass
