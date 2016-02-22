#!/bin/bash

rosbag record -a -x "(.*)raw|(.)*Depth|(.)*theora"
