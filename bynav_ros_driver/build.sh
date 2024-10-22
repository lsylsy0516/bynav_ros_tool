#!/bin/bash

ROS_VERSION=`printenv ROS_DISTRO`
echo "ROS version: ${ROS_VERSION}"

if [ -z $ROS_VERSION ]; then
  echo "ROS env not setup"
  exit
fi

catkin_make --source bynav_ros_driver
