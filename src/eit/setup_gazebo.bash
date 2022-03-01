#!/bin/bash

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/PX4-Autopilot/Tools/sitl_gazebo
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/master_thesis/src/asv_wave_sim/asv_wave_sim_gazebo/models

echo ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH
