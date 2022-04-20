source ~/.bashrc
file_path=`realpath $0`
dir_path=`dirname $file_path`
rest_path=catkin_ws/devel/setup.bash
source "$dir_path/$rest_path"

export TURTLEBOT3_MODEL=burger
# export ROS_IP=$(ifconfig | grep -Eo 'inet adr:([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1')

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

gnome-terminal -e "roslaunch turtlebot3_gazebo turtlebot3_autorace.launch"
sleep 3
gnome-terminal -e "roslaunch turtlebot3_autorace_camera turtlebot3_autorace_intrinsic_camera_calibration.launch"
sleep 3
gnome-terminal -e "roslaunch turtlebot3_autorace_core turtlebot3_autorace_core.launch"
sleep 3

python3 RoadMap_Visualisor/Pos.py > /dev/null 2>&1 &
sleep 1
# python3 RoadMap_Visualisor/MainWindow.py

# roslaunch turtlebot3_gazebo turtlebot3_autorace.launch > /dev/null 2>&1 &
# sleep 3
# roslaunch turtlebot3_autorace_camera turtlebot3_autorace_intrinsic_camera_calibration.launch > /dev/null 2>&1 &
# sleep 3
# roslaunch turtlebot3_autorace_core turtlebot3_autorace_core.launch > /dev/null 2>&1 &
# sleep 3

# python3 RoadMap\ Visualisor/Pos.py > /dev/null 2>&1 &
# sleep 1
# python3 RoadMap\ Visualisor/MainWindow.py
cat


trap_ctrlc