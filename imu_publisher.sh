#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_IP=192.168.1.236
export ROS_MASTER_URI=http://192.168.1.120:11311
chmod +x /home/nvidia/catkin_ws/src/imu_preintegration_ros/scripts/IMU_publisher.py
rosrun imu_preintegration_ros IMU_publisher.py