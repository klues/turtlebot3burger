#!/usr/bin/env python

"""
Node 'scanAnalyzeNode.py'

Analyzes topic /scan for closest and farthest obstacle and publishes
topic /scanAnalyzed.
"""

import rospy
import math
import numpy
from sensor_msgs.msg import LaserScan
from turtlebot3burger.msg import ScanAnalyzed

pubScanAnalyzed = rospy.Publisher('scanAnalyzed', ScanAnalyzed, queue_size=10)

# how much values in front/back are used to calculate front/back distance?
meanCount = 30

# range in degrees for analysing minimum values in front/back
frontBackRange = 90

def scanAnalyzeNode():
    
    rospy.init_node('scanAnalyzeNode', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scanCallback)
    
    rospy.spin() #keeps script running until node terminated

def scanCallback(data):
    """ Callback for data coming from topic /scan
    
    :param data: data of type sensor_msgs/LaserScan
    """
    
    (minValue, minAngle, maxValue, maxAngle) = getMinMaxRange(data, 0, len(data.ranges))
    halfRange = frontBackRange / 2
    halfLength = len(data.ranges) / 2
    (minValueFront, minAngleFront) = getMinRange(data, 0, halfRange - 1, 360 - halfRange, 360)
    (minValueBack, minAngleBack) = getMinRange(data, halfLength - halfRange, halfLength + halfRange - 1)
    
    # calculate front/back distances
    length = len(data.ranges)
    halfMeanCount = meanCount / 2
    frontRanges = getRange(data, 0, halfMeanCount - 1, 360 - halfMeanCount, 360)
    backRanges = getRange(data, halfLength - halfMeanCount, halfLength + halfMeanCount - 1)
    backRange = numpy.mean(backRanges)
    frontRange = numpy.mean(frontRanges)
    
    msg = ScanAnalyzed()
    msg.range_min = minValue
    msg.range_max = maxValue
    msg.range_front = frontRange
    msg.range_front_min = minValueFront
    msg.range_back = backRange
    msg.range_back_min = minValueBack
    msg.angle_range_min = minAngle
    msg.angle_range_max = maxAngle
    msg.angle_front_min = minAngleFront
    msg.angle_back_min = minAngleBack
    
    pubScanAnalyzed.publish(msg)
    

def getRangeWithAngles(data, fromIndex, toIndex, fromIndex2 = None, toIndex2 = None):
    """ Extracts scan data within a specific angle range. Returns two 
    arrays: range data and corresponding angles. Possible to pass two
    ranges which should be returned (e.g. 0-45 and 315-360 in order to
    retrieve 90 degree range in front of robot).
    
    :param data: data of type sensor_msgs/LaserScan
    :param fromIndex: starting index of range to return
    :param toIndex: end index of range to return
    :param fromIndex2: (optional) start index of second range to return
    :param toIndex2: (optional) end index of second range to return
    :return (ranges, angles):   two array containing corresponding ranges
                                and angles in degrees
    """
    resultRanges = []
    resultAngles = []
    for idx, val in enumerate(data.ranges):
        inRange1 = idx >= fromIndex and idx <= toIndex
        inRange2 = toIndex2 and (idx >= fromIndex2 and idx <= toIndex2)
        if inRange1 or inRange2:
            resultRanges.append(val)
            resultAngles.append(idx * data.angle_increment * 180 / math.pi)
    
    return (resultRanges, resultAngles)
    
def getRange(data, fromIndex, toIndex, fromIndex2 = None, toIndex2 = None):
    """ same as getRangeWithAngles(), but only ranges are returned, no angles
    """
    return getRangeWithAngles(data, fromIndex, toIndex, fromIndex2, toIndex2)[0]

def getMinMaxRange(data, fromIndex, toIndex, fromIndex2 = None, toIndex2 = None):
    """ returns minimum and maximum values and corresponding angles within
    a specific range.
    :param data: data of type sensor_msgs/LaserScan
    :param fromIndex: starting index of range to analyze
    :param toIndex: end index of range to analyze
    :param fromIndex2: (optional) start index of second range to analyze
    :param toIndex2: (optional) end index of second range to analyze
    :return (minValue, minAngle, maxValue, maxAngle): minimum and maximum
            values within given ranges with corresponding angles in degrees
    
    """
    (resultRanges, resultAngles) = getRangeWithAngles(data, fromIndex, toIndex, fromIndex2, toIndex2)
    maxValue = max(resultRanges)
    validValues = [i for i in resultRanges if i > 0]
    minValue = min(validValues) if len(validValues) > 0 else 0
    minValueIndex = resultRanges.index(minValue)
    maxValueIndex = resultRanges.index(maxValue)
    minAngle = resultAngles[minValueIndex]
    maxAngle = resultAngles[maxValueIndex]
    
    return (minValue, minAngle, maxValue, maxAngle)
    
def getMinRange(data, fromIndex, toIndex, fromIndex2 = None, toIndex2 = None):
    """ same as getMinMaxRange() but returns only (minValue, minAngle)
    
    """
    result = getMinMaxRange(data, fromIndex, toIndex, fromIndex2, toIndex2)
    return (result[0], result[1])
    
if __name__ == '__main__':
    try:
        scanAnalyzeNode()
    except rospy.ROSInterruptException:
        pass
