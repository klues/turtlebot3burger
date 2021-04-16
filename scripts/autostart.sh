#!/bin/bash

pkill roslaunch

source /opt/ros/kinetic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash

export TURTLEBOT3_MODEL=burger
OWN_IP=$(hostname -I | xargs)

roslaunch turtlebot3burger startup.launch &

while [ ! "$OWN_IP" ]; do
  echo "network interface not up, will try again in 1 second";
  sleep 1;
  OWN_IP=$(hostname -I | xargs)
done

pkill roslaunch
sleep 5

export ROS_HOSTNAME=$OWN_IP
echo $ROS_HOSTNAME

roslaunch turtlebot3burger startup.launch
