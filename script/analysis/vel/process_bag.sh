#!/usr/bin/env bash

FILES=`find $1 -name "*.bag"`
for BAG in $FILES
do
  echo $BAG
  filename="${BAG##*/}"
  echo $filename
  python post_process.py $BAG
  BAG_P=${filename%.*}_processed.bag
  echo $BAG_P
  rostopic echo -b $BAG_P -p /vservo/cmd_vel > $2/${filename%.*}_vservo.csv
  rostopic echo -b $BAG_P -p /bebop/states/ARDrone3/PilotingState/SpeedChanged > $2/${filename%.*}_speed.csv
  rostopic echo -b $BAG_P -p /vicon/bebop_blue_en/bebop_blue_en > $2/${filename%.*}_vicon.csv
  rostopic echo -b $BAG_P -p /bebop/states/ARDrone3/PilotingState/AttitudeChanged > $2/${filename%.*}_att.csv
  rm $BAG_P
done
