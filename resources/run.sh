#!/bin/bash
ROS_WORKSPACE=/app/ros

cd $ROS_WORKSPACE
source devel/setup.bash
roslaunch launch/styx.launch
