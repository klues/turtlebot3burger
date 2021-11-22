#!/bin/bash

pkill roslaunch

source /opt/ros/kinetic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash

export TURTLEBOT3_MODEL=burger
# see https://stackoverflow.com/questions/8529181/which-terminal-command-to-get-just-ip-address-and-nothing-else
OWN_IP=$(ip -4 addr show wlan0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}')

while [ ! "$OWN_IP" ]; do
  echo "network interface not up, will try again in 1 second";
  sleep 1;
  OWN_IP=$(ip -4 addr show wlan0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}')
done

export ROS_HOSTNAME=$OWN_IP
echo $ROS_HOSTNAME

roslaunch turtlebot3burger startup.launch
