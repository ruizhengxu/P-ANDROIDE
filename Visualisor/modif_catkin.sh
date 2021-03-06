#!/bin/bash

# Gazebo map env
rm catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_autorace.world
cp .stuff/turtlebot3_autorace.world catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_autorace.world

# Gazebo map object (dimensions)
rm catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_autorace/course/model.sdf
cp .stuff/model.sdf catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_autorace/course/model.sdf

# Robot default position and gui
rm catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_autorace.launch
cp .stuff/turtlebot3_autorace.launch catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_autorace.launch

# Robot speeding rules
rm catkin_ws/src/turtlebot3_autorace/turtlebot3_autorace_control/nodes/control_lane
cp .stuff/control_lane catkin_ws/src/turtlebot3_autorace/turtlebot3_autorace_control/nodes/control_lane

# Robot physics
rm catkin_ws/src/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger_for_autorace.gazebo.xacro
cp .stuff/turtlebot3_burger_for_autorace.gazebo.xacro catkin_ws/src/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger_for_autorace.gazebo.xacro

# make control lane executable
chmod 777 visualisor.app
chmod 777 setup.sh
chmod 777 start.sh
chmod 777 quit.sh
chmod 777 catkin_ws/src/turtlebot3_autorace/turtlebot3_autorace_control/nodes/control_lane
chmod 777 catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_autorace.launch
