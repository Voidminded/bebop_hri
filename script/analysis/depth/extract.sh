#!/usr/bin/env bash

#Takes 2 arguments as input, first argument is the path to folder containing all the bags that should be proccessed, second argument is the path to destination folder for saving all the CSV files.

FILES=`find $1 -name "*.bag"`
for BAG in $FILES
do
  echo $BAG
  filename="${BAG##*/}"
  echo $filename
  rostopic echo -b $BAG -p /vservo/debug > $2/${filename%.*}_vservo.csv
  rostopic echo -b $BAG -p /vicon/bebop_blue_en/bebop_blue_en > $2/${filename%.*}_bebop.csv
  rostopic echo -b $BAG -p /vicon/bebop_target/bebop_target > $2/${filename%.*}_target.csv
done
