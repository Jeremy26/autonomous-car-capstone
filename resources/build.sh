#!/bin/bash
ROS_WORKSPACE=/app/ros

if [ ! -d $ROS_WORKSPACE/build ]; then
    mkdir -p $ROS_WORKSPACE/build
fi

if [ ! -d $ROS_WORKSPACE/devel ]; then
    mkdir -p $ROS_WORKSPACE/devel
fi

cd $ROS_WORKSPACE
rosdep init && rosdep update
catkin_make
