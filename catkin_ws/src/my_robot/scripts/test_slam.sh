#!/bin/sh

xterm -e "export TURTLEBOT3_MODEL=waffle_pi;
roslaunch my_robot world.launch" &

sleep 3

xterm -e "export TURTLEBOT3_MODEL=waffle_pi;
roslaunch my_robot slam.launch slam_methods:=gmapping" &

sleep 3

xterm  -e "export TURTLEBOT3_MODEL=waffle_pi;
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch"

