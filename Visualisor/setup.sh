#!/bin/bash

source ~/.bashrc
source catkin_ws/devel/setup.bash

export TURTLEBOT3_MODEL=burger
export GAZEBO_MODE=true
export AUTO_IN_CALIB=action
export AUTO_EX_CALIB=action
export AUTO_DT_CALIB=action

function trap_ctrlc(){
    killall -9 gzclient
    killall -9 gzserver
    pkill -f Pos.py
    pkill -f MainWindow.py
    rm -f data.json
    exit 2
}

trap "trap_ctrlc" 2

sleep 3
gnome-terminal -e "roslaunch turtlebot3_gazebo turtlebot3_autorace.launch"
sleep 3
gnome-terminal -e "roslaunch turtlebot3_autorace_camera turtlebot3_autorace_intrinsic_camera_calibration.launch"
sleep 3
gnome-terminal -e "roslaunch turtlebot3_autorace_core turtlebot3_autorace_core.launch"

cat
trap_ctrlc