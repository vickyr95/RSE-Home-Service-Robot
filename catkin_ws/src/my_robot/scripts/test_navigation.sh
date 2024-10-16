#!/bin/sh

xterm -e "export TURTLEBOT3_MODEL=waffle_pi;
roslaunch my_robot world.launch" &

sleep 3

xterm -e "export TURTLEBOT3_MODEL=waffle_pi; 
roslaunch my_robot navigation.launch map_file:=$(rospack find my_robot)/maps/map.yaml"
