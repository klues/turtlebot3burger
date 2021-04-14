#!/usr/bin/env python

"""
Node 'autoFollowNode.py'

Rotates the robot in direction of the closest obstacle in front of the robot.
Follows/eludes the obstacle by maintaining a constant distance.
"""

import rospy


def autoFollowNode():
    print("starting autoFollowNode...")
	# TODO
    

if __name__ == '__main__':
    try:
        autoFollowNode()
    except rospy.ROSInterruptException:
        pass
