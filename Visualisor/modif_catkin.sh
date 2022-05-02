#!/bin/bash

rm catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_autorace.world
cp .stuff/turtlebot3_autorace.world catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_autorace.world

rm catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_autorace/course/model.sdf
cp .stuff/model.sdf catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_autorace/course/model.sdf

rm catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_autorace.launch
cp .stuff/turtlebot3_autorace.launch catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_autorace.launch