export TURTLEBOT3_MODEL=burger
export ROS_IP=$(ifconfig | grep -Eo 'inet adr:([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1')
source ~/.bashrc

function trap_ctrlc(){
    killall -9 gzclient
    killall -9 gzserver
    pkill -f Pos.py
    pkill -f MainWindow.py
    rm -f data.json
    exit 2
}

trap "trap_ctrlc" 2

# Starting ros with gazebo link
roslaunch turtlebot3_gazebo turtlebot3_autorace.launch > /dev/null 2>&1 &
sleep 3

# Recovery position throught odometry node with a new tty
# gnome-terminal -e "python pos.py"

python3 Pos.py > /dev/null 2>&1 &
sleep 1
gnome-terminal -e "python3 MainWindow.py"

# Launch Keyboard inputs
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
trap_ctrlc
