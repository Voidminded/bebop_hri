#!/bin/bash

roslaunch bebop_tools bebop_nodelet_iv.launch &
sleep 5
roslaunch bebop_tools joy_teleop.launch &
sleep 5
roslaunch vicon_bridge vicon.launch &
sleep 5

rosparam dump `date +"%F-%T" | sed "s/\:/\-/g"`.yaml

rosbag record -a -x "(.*)raw|(.)*Depth|(.)*theora|(.)*compressed" &

sleep 5

#rostopic pub --once /bebop/camera_control geometry_msgs/Twist  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0, y: -10.0, z: 0.0}}'

echo "Ready!"
