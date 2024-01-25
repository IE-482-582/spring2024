#!/bin/bash

# Refresh .bashrc to get the appropriate/up-to-date ROS IP addresses.
# It seems weird to do this, but sometimes this script hangs if our
# IP address has changed since when we started ROS.
source ${HOME}/.bashrc
sleep 2s

# Stop all running nodes:
rosnode kill --all

# Stop roscore:
killall roscore
killall rosout
killall roslaunch
killall rosmaster


# Sadly, the above steps often hang.  
# Here we employ the nuclear option:
sleep 2s
for cmd in "roscore" \
		   "rosout"  \
		   "roslaunch"  \
		   "rosmaster"  \
		   "gzclient"  \
		   "gzserver"  
do
	CNT=$(pidof $cmd | wc -w)
	# if [ $myIP != "$i" ]; then
	if (($CNT>0)); then
		echo "Killing $cmd"
		kill -9 $(pidof $cmd)
	fi
done


CNT=$(pidof python | wc -w)
if (($CNT>0)); then
	echo "NOTE: There are $CNT python processes running"
	echo "FIXME -- Give option to kill these?"
	# kill -9 $(pidof $cmd)
fi
 

echo "All ros nodes and roscore should now be stopped."
