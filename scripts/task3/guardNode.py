#!/usr/bin/env python

"""
Node 'guardNode.py'

Passes topic /cmd_vel_raw to /cmd_vel if there is no obstacle detected
by LIDAR of Turtlebot. Additionally plays a beeping sound when close to
obstacles.
"""

import rospy


def guardNode():
    print("starting guardNode...")
	# TODO


if __name__ == '__main__':
    try:
        guardNode()
    except rospy.ROSInterruptException:
        pass
