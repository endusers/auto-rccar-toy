#!/bin/bash

CURRENT_DIR=`pwd`

cd `dirname $0`
SCRIPTS_DIR=`pwd`
DIR=`dirname ${SCRIPTS_DIR}`
echo ${DIR}

WORKSPACE=~/rccar_ws
if [ $# -ge 1 ]
then
    WORKSPACE=$1
fi

cd ${CURRENT_DIR}
mkdir -p ${WORKSPACE}/src
cd ${WORKSPACE}
WORKSPACE=`pwd`
echo ${WORKSPACE}

# bno055
cd ${WORKSPACE}/src
git clone -b main 'https://github.com/flynneva/bno055.git'
cd ${WORKSPACE}/src/bno055
git checkout -B main 727d4abc937536be9ad33905dff2a2bde9c3b220

# Groot
cd ${WORKSPACE}/src
git clone -b master 'https://github.com/BehaviorTree/Groot.git'
cd ${WORKSPACE}/src/Groot
git checkout -B master 89ba0533c1177d90b0e9dc142734f83a206776c4
cd ${WORKSPACE}/src/Groot/depend
git clone -b v3.8 'https://github.com/BehaviorTree/BehaviorTree.CPP.git'
cd ${WORKSPACE}/src/Groot/depend/BehaviorTree.CPP
git checkout -B v3.8 c07223a6d065666bb19e34d085b04684dd424503

# navigation2
cd ${WORKSPACE}/src
git clone -b foxy-devel 'https://github.com/ros-planning/navigation2.git'
cd ${WORKSPACE}/src/navigation2
git checkout -B foxy-devel ca482808a7a7c52ce01ae3c662dc2b980968fc16

# realsense_gazebo_plugin
cd ${WORKSPACE}/src
git clone -b foxy-devel 'https://github.com/pal-robotics/realsense_gazebo_plugin.git'
cd ${WORKSPACE}/src/realsense_gazebo_plugin
git checkout -B foxy-devel 73b3061c5a97fc5846112b0ea2da70c6e24d9782

# realsense-ros
cd ${WORKSPACE}/src
git clone -b ros2-development 'https://github.com/IntelRealSense/realsense-ros.git'
cd ${WORKSPACE}/src/realsense-ros
git checkout -B ros2-development 333f4fbbc3ddde86cd4d1896e02bfc9d7f550ba4

# robot_localization
cd ${WORKSPACE}/src
git clone -b foxy-devel 'https://github.com/cra-ros-pkg/robot_localization.git'
cd ${WORKSPACE}/src/robot_localization
git checkout -B foxy-devel a9c96ebc245575adc06cea4dad5578a3ca3bdb02

# rviz_satellite
cd ${WORKSPACE}/src
git clone -b ros2 'https://github.com/nobleo/rviz_satellite.git'
cd ${WORKSPACE}/src/rviz_satellite
git checkout -B ros2 63900f348fd518f25682d1b5c4551ba898baf035

# slam_toolbox
cd ${WORKSPACE}/src
git clone -b foxy-devel 'https://github.com/SteveMacenski/slam_toolbox.git'
cd ${WORKSPACE}/src/slam_toolbox
git checkout -B foxy-devel 4786e90c06a4dc6fa811c5057d4e88387fba3829

# ublox
cd ${WORKSPACE}/src
git clone -b foxy-devel 'https://github.com/KumarRobotics/ublox.git'
cd ${WORKSPACE}/src/ublox
git checkout -B foxy-devel 691dec4166444d151691fae2d5f1775ddc1cf825

# uros
cd ${WORKSPACE}/src
mkdir uros
cd ${WORKSPACE}/src/uros
git clone -b master 'https://github.com/micro-ROS/drive_base.git'
cd ${WORKSPACE}/src/uros/drive_base
git checkout -B master fc55ef35fbdb99c999ad6f9d5b5c6b6968093376
cd ${WORKSPACE}/src/uros
git clone -b foxy 'https://github.com/micro-ROS/micro-ROS-Agent.git'
cd ${WORKSPACE}/src/uros/micro-ROS-Agent
git checkout -B foxy 096d54a5e912510c7ae698cb263dcf280b472a4b
cd ${WORKSPACE}/src/uros
git clone -b foxy 'https://github.com/micro-ROS/micro_ros_msgs.git'
cd ${WORKSPACE}/src/uros/micro_ros_msgs
git checkout -B foxy e3664463e78ae5d0c34d86be92d707b3d9dfd27d

cd ${WORKSPACE}/src

# auto-rccar-toy
cp -rf ${DIR} ${WORKSPACE}/src/auto-rccar-toy

# update-src
unzip -o -d ${WORKSPACE}/src ${DIR}/scripts/update-src/bno055.zip
unzip -o -d ${WORKSPACE}/src ${DIR}/scripts/update-src/navigation2.zip
unzip -o -d ${WORKSPACE}/src ${DIR}/scripts/update-src/realsense-ros.zip

# build
cd ${WORKSPACE}
rosdep update
rosdep install -y --from-paths ./src --ignore-src
colcon build --symlink-install
