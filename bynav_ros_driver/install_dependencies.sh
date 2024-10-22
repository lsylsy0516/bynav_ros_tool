#!/bin/bash

INSTALLATION_METHOD=$1

function print_usage() {
  echo "usage: ./install_dependencies.sh method"
  echo "method: online 或者 offline"
}

function install_online() {
  echo "install_online"
  sudo apt install libgps-dev
}

function install_offline() {
  echo "install_offline"
  dir_name="dependencies/"$1
  if [ $1 = "20.04" ]; then
    sudo dpkg -i $dir_name"/libgps26_3.20-8ubuntu0.4_amd64.deb"
    sudo dpkg -i $dir_name"/libgps-dev_3.20-8ubuntu0.4_amd64.deb"
  else
    sudo dpkg -i $dir_name"/libgps23_3.17-5_amd64.deb"
    sudo dpkg -i $dir_name"/libgps-dev_3.17-5_amd64.deb" 
  fi
}

if [ -z $INSTALLATION_METHOD ]; then
  print_usage
  exit
fi

if [ $INSTALLATION_METHOD != "online" -a $INSTALLATION_METHOD != "offline" ]; then
  print_usage
  exit
fi

ROS_VERSION=`printenv ROS_DISTRO`
echo "ROS version: ${ROS_VERSION}"
if [ -z $ROS_VERSION ]; then
  echo "ROS env not setup"
  exit
fi

if [ $INSTALLATION_METHOD = "online" ]; then
  install_online
else
  ubuntu_release=`lsb_release -r | awk '{print $2}'`
  install_offline $ubuntu_release
fi
