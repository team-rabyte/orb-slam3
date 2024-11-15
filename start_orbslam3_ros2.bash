#!/bin/bash

export LD_LIBRARY_PATH=/home/nvidia/projects/ORB_SLAM3/lib:$LD_LIBRARY_PATH
source ./Examples/ROS/ORB_SLAM3/install/setup.bash
ros2 run orb_slam3_ros2 mono ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml
