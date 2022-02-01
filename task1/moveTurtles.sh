SCRIPTDIR=$(dirname "$0")
sh $SCRIPTDIR/stop.sh
sh $SCRIPTDIR/start.sh
sleep 3

rosservice call /spawn 1 3 0 "test"
#rosservice call /spawn "{x: 1.0, y: 3.0, theta: 0.0, name: 'test'}"

# TODO: change / insert your commands
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist --once -- '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]' &
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0.5]' &
