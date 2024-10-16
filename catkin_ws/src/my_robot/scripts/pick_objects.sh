#!/bin/sh

xterm -e "export TURTLEBOT3_MODEL=waffle_pi;
roslaunch my_robot world.launch" &

sleep 3

xterm -e "export TURTLEBOT3_MODEL=waffle_pi; 
roslaunch my_robot navigation.launch map_file:=/home/frlinux-18-04/catkin_ws/src/my_robot/maps/map.yaml" &

sleep 3

xterm -e "rosrun my_robot pick_objects.py "
