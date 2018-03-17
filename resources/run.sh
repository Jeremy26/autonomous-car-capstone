#!/bin/bash
ROS_WORKSPACE=/app/ros

cd $ROS_WORKSPACE
source devel/setup.bash
rm -Rf /root/.log/*
roslaunch launch/styx.launch
