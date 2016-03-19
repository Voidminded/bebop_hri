#!/usr/bin/env bash

FILES=`find $1 -name "*.bag"`
for BAG in $FILES
do
  echo $BAG
  filename="${BAG##*/}"
  echo $filename
  rostopic echo -b $BAG -p /behavior/status > $2/${filename%.*}_status.csv
  rostopic echo -b $BAG -p /bebop/states/ARDrone3/PilotingState/PositionChanged > $2/${filename%.*}_gps.csv
done
