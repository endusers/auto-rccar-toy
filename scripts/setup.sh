#!/bin/bash

WORKSPACE=~/rccar_ws
INSTALL_RTKLIB=OFF
while getopts w:r- OPT
do
    if [ ${OPT} = w ]
    then
        WORKSPACE=${OPTARG}
    elif [ ${OPT} = r ]
    then
        INSTALL_RTKLIB=ON
    else
        break
    fi
done
shift `expr ${OPTIND} - 1`

CURRENT_DIR=`pwd`

cd `dirname $0`
SCRIPTS_DIR=`pwd`
DIR=`dirname ${SCRIPTS_DIR}`
echo ${DIR}

cd ${CURRENT_DIR}
mkdir -p ${WORKSPACE}/src
cd ${WORKSPACE}
WORKSPACE=`pwd`
echo ${WORKSPACE}

# bno055
cd ${WORKSPACE}/src
git clone -b main 'https://github.com/flynneva/bno055.git'
cd ${WORKSPACE}/src/bno055
git checkout -B main b20cd4e027c5f07db1248d8f7008ddb97cbb471d

# Groot
cd ${WORKSPACE}/src
git clone -b master 'https://github.com/BehaviorTree/Groot.git'
cd ${WORKSPACE}/src/Groot
git checkout -B master ca6c8f253f033bbdbe9294d1c9d0ac0beeb00241
cd ${WORKSPACE}/src/Groot/depend
git clone -b v3.8 'https://github.com/BehaviorTree/BehaviorTree.CPP.git'
cd ${WORKSPACE}/src/Groot/depend/BehaviorTree.CPP
git checkout -B v3.8 04a514eecc0e02e38f8c104084f43a85f57fabd2

# navigation2
cd ${WORKSPACE}/src
git clone -b humble 'https://github.com/ros-planning/navigation2.git'
cd ${WORKSPACE}/src/navigation2
git checkout -B humble 3ed4c2dfa1ef9b31e117ccb5c35486b599e6b97e

# realsense_gazebo_plugin
cd ${WORKSPACE}/src
git clone -b foxy-devel 'https://github.com/pal-robotics/realsense_gazebo_plugin.git'
cd ${WORKSPACE}/src/realsense_gazebo_plugin
git checkout -B foxy-devel 73b3061c5a97fc5846112b0ea2da70c6e24d9782

# realsense-ros
cd ${WORKSPACE}/src
git clone -b ros2-master 'https://github.com/IntelRealSense/realsense-ros.git'
cd ${WORKSPACE}/src/realsense-ros
git checkout -B ros2-master 1cbd81be81e807eefb46f098e76381888ffc7001

# robot_localization
cd ${WORKSPACE}/src
git clone -b humble-devel 'https://github.com/cra-ros-pkg/robot_localization.git'
cd ${WORKSPACE}/src/robot_localization
git checkout -B humble-devel 72f9e936062a2a6b9a210a36ba5e554058191a92

# rviz_satellite
cd ${WORKSPACE}/src
git clone 'https://github.com/nobleo/rviz_satellite.git'
cd ${WORKSPACE}/src/rviz_satellite
git checkout -b ros2 995b4090ff1f770df2f7c4bb4bdb6b5809fa66b1

# slam_toolbox
cd ${WORKSPACE}/src
git clone -b humble 'https://github.com/SteveMacenski/slam_toolbox.git'
cd ${WORKSPACE}/src/slam_toolbox
git checkout -B humble 6f34357ec8dc20dec46f26dbf6837f1baa70ded4

# ublox
cd ${WORKSPACE}/src
git clone -b ros2 'https://github.com/KumarRobotics/ublox.git'
cd ${WORKSPACE}/src/ublox
git checkout -B ros2 577ef65095bdc5f2cd1a2ea32a4dbf13dbcd0b7c

# uros
cd ${WORKSPACE}/src
mkdir uros
cd ${WORKSPACE}/src/uros
git clone -b master 'https://github.com/micro-ROS/drive_base.git'
cd ${WORKSPACE}/src/uros/drive_base
git checkout -B master fc55ef35fbdb99c999ad6f9d5b5c6b6968093376
cd ${WORKSPACE}/src/uros
git clone -b humble 'https://github.com/micro-ROS/micro-ROS-Agent.git'
cd ${WORKSPACE}/src/uros/micro-ROS-Agent
git checkout -B humble 1b815304e9432bb843d7258d8e38594954f79bab
cd ${WORKSPACE}/src/uros
git clone -b humble 'https://github.com/micro-ROS/micro_ros_msgs.git'
cd ${WORKSPACE}/src/uros/micro_ros_msgs
git checkout -B humble 9f9ab03b5d7a25fd9ff0c6df4f11838905d30ca5

# rtabmap
cd ${WORKSPACE}/src
git clone -b master 'https://github.com/introlab/rtabmap.git'
cd ${WORKSPACE}/src/rtabmap
git checkout -B master 1ab0133f140a7de6c9e4b077bb9d5bdf82bf5942
cd ${WORKSPACE}/src
git clone -b ros2 'https://github.com/introlab/rtabmap_ros.git'
cd ${WORKSPACE}/src/rtabmap_ros
git checkout -B ros2 fdd13c31f9574e72ed27a79321dfa00676176239

# livox_ros_driver2
cd ${WORKSPACE}/src
git clone -b 1.2.4 'https://github.com/Livox-SDK/livox_ros_driver2.git'

# livox_laser_simulation
cd ${WORKSPACE}/src
git clone -b main 'https://github.com/LihanChen2004/livox_laser_simulation_ros2.git'
cd ${WORKSPACE}/src/livox_laser_simulation_ros2
git checkout -B main cee09dc9eea6e0a9822735bbc98c925441bbb019

# FAST_LIO
cd ${WORKSPACE}/src
git clone -b ROS2 https://github.com/hku-mars/FAST_LIO.git --recurse-submodules
cd ${WORKSPACE}/src/FAST_LIO
git checkout -B ROS2 a4743b095409588842a5b30ddfa27e29d2f99164 --recurse-submodules

cd ${WORKSPACE}/src

# auto-rccar-toy
cp -rf ${DIR} ${WORKSPACE}/src/auto-rccar-toy

# update-src
cp -rf ${DIR}/scripts/update-src/realsense-ros ${WORKSPACE}/src
cp -rf ${DIR}/scripts/update-src/navigation2 ${WORKSPACE}/src
cp -rf ${DIR}/scripts/update-src/livox_ros_driver2 ${WORKSPACE}/src
rm -rf ${WORKSPACE}/src/auto-rccar-toy/scripts/update-src

# not support gazebo-ros-pkgs
if [ $(uname -m) != "x86_64" ]
then
    rm -rf ${WORKSPACE}/src/realsense_gazebo_plugin
    rm -rf ${WORKSPACE}/src/navigation2/nav2_system_tests
    rm -rf ${WORKSPACE}/src/livox_laser_simulation_ros2
fi

# build
cd ${WORKSPACE}
rosdep update
rosdep install -y --from-paths ./src --ignore-src
colcon build --symlink-install

# RTKLIB
if [ ${INSTALL_RTKLIB} = ON ]
then
    cd ~/
    git clone -b rtklib_2.4.3 'https://github.com/tomojitakasu/RTKLIB.git'
    cd ~/RTKLIB/lib/iers/gcc/
    make
    cd ~/RTKLIB/app/consapp
    make

    cp -f ${DIR}/scripts/rtklib-str2str.sh ${WORKSPACE}
fi
