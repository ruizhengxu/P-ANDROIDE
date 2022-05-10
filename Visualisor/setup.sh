#!/bin/bash

source ~/.bashrc
source catkin_ws/devel/setup.bash

export TURTLEBOT3_MODEL=burger
export GAZEBO_MODE=true
export AUTO_IN_CALIB=action
export AUTO_EX_CALIB=action
export AUTO_DT_CALIB=action

function trap_ctrlc(){
    ./quit.sh
}

trap "trap_ctrlc" 2

if  [[ $1 = "-d" ]]; then
    sleep 1
    gnome-terminal -e "roslaunch turtlebot3_gazebo turtlebot3_autorace.launch"
    sleep 1
    gnome-terminal -e "roslaunch turtlebot3_autorace_camera turtlebot3_autorace_intrinsic_camera_calibration.launch"
    sleep 1
    gnome-terminal -e "roslaunch turtlebot3_autorace_core turtlebot3_autorace_core.launch"
else
    sleep 1
    roslaunch turtlebot3_gazebo turtlebot3_autorace.launch > /dev/null 2>&1 &
    sleep 1
    roslaunch turtlebot3_autorace_camera turtlebot3_autorace_intrinsic_camera_calibration.launch > /dev/null 2>&1 &
    sleep 1
    roslaunch turtlebot3_autorace_core turtlebot3_autorace_core.launch > /dev/null 2>&1 &
fi
python3 RoadMap_Visualisor/Pos.py > /dev/null 2>&1 &