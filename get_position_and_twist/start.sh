function trap_ctrlc(){
    killall -9 gzclient
    killall -9 gzserver
    pkill -f pos.py
    exit 2
}

trap "trap_ctrlc" 2

export TURTLEBOT3_MODEL=burger

# Starting ros with gazebo link
roslaunch turtlebot3_gazebo turtlebot3_autorace.launch > /dev/null 2>&1 &
sleep 2

# Recovery position throught odometry node with a new tty
gnome-terminal -e "python pos.py"

# Launch Keyboard inputs
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
trap_ctrlc
