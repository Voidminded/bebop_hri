#!/bin/bash

rosparam dump `date +"%F-%T" | sed "s/\:/\-/g"`.yaml
rosbag record -a -x "(.*)raw|(.)*Depth|(.)*theora|(.*)debug_image(.*)"

echo "Done!"
