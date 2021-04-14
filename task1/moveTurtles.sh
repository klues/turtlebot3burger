sh stop.sh
sh start.sh
sleep 3

# TODO: change / insert your commands
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0.5]' &
