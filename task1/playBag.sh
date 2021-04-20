SCRIPTDIR=$(dirname "$0")
sh $SCRIPTDIR/stop.sh
sh $SCRIPTDIR/start.sh
sleep 3

# TODO insert rosbag play command to play your bagfile
