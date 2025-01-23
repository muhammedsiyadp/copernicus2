#!/bin/bash

# Ask the user for input
echo "Do you want to launch with IMU and EKF or just odom without IMU? (y/n)"
read response
echo "Do you want to launch Rviz? (y/n)"
read rviz_launch

source /opt/ros/noetic/setup.bash
source ~/copernicus2/devel/setup.bash

# Check the response and execute the appropriate command
if [[ "$response" == "y" ]]; then
    echo "Launching with EKF_IMU..."
    if [[ "$rviz_launch" == "y" ]]; then
        roslaunch copernicus_base bringup_imu.launch use_rviz:=true
    else
        roslaunch copernicus_base bringup_imu.launch
    fi
else
    echo "Launching without EKF_IMU..."
    if [[ "$rviz_launch" == "y" ]]; then
        roslaunch copernicus_base bringup.launch use_rviz:=true
    else
        roslaunch copernicus_base bringup.launch
    fi
fi
