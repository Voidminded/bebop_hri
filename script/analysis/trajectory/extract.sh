#!/usr/bin/env bash

FILES=`find $1 -name "*.bag"`
for BAG in $FILES
do
  echo $BAG
  filename="${BAG##*/}"
  echo $filename
  rostopic echo -b $BAG -p /behavior/status > $2/${filename%.*}_status.csv
  rostopic echo -b $BAG -p /bebop/land > $2/${filename%.*}_land.csv
  rostopic echo -b $BAG -p /vicon/bebop_blue_en/bebop_blue_en > $2/${filename%.*}_bebop.csv
  rostopic echo -b $BAG -p /vicon/bebop_target/bebop_target > $2/${filename%.*}_target.csv
  python trimmer.py $2/${filename%.*}
done
