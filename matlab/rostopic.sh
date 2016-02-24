#!/usr/bin/env bash
rostopic echo -b $1 -p /bebop/states/ARDrone3/PilotingState/AttitudeChanged > $1_att.csv
rostopic echo -b $1 -p /bebop/cmd_vel_stamped > $1_cmdvel.csv
rostopic echo -b $1 -p vicon/bebop_blue_en/bebop_blue_en > $1_vicon.csv
rostopic echo -b $1 -p /bebop/states/ARDrone3/PilotingState/SpeedChanged > $1_speed.csv

