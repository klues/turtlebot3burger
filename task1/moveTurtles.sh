SCRIPTDIR=$(dirname "$0")
sh $SCRIPTDIR/stop.sh
sh $SCRIPTDIR/start.sh
sleep 3

# TODO: change / insert your commands
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist --once -- '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]' &
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0.5]' &
