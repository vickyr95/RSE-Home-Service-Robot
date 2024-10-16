# Udacity RSE-Home-Service-Robot

## Description
  This is a robotics simulation project to simulate a home service robot capable of navigating to pick up and deliver virtual objects. 
  
  The project uses [turtlebot3](https://wiki.ros.org/turtlebot3) for simulation. The gazebo world is created using gazebo building editor.

  The goal of the project is for learning the robotics concepts like mapping, localisation, slam and navigation and apply in a project to gain better understanding.
  
  Below is the list of steps to make this work.
  - Build a simulation environment.
  - Build a map of the simulated environment using any mapping algorithm (gmapping) and save the map.
  - Use a localisation algorithm to let the robot know where it is located with respect to the known map.
  - create a pick_objects.sh file that will send multiple goals for the robot to reach. The robot travels to the desired pickup zone, displays a message that it reached its destination, waits 5 seconds, travels to the desired drop off zone, and displays a message that it reached the drop off zone.
  - write a home_service.sh file that will run all the nodes in this project.
  - write a add_marker.sh file that will publish a marker to rviz. Initially show the marker at the pickup zone. Hide the marker once your robot reach the pickup zone. Wait 5 seconds to simulate a pickup. Show the marker at the drop off zone once your robot reaches it.

## Project Home Service Bot Demo
![RSE-Project](https://github.com/user-attachments/assets/7b16a544-79bd-4ae7-a1a0-8cf277a7ac2c)

### To run this project

* Clone this repository.
```
git clone https://github.com/vickyr95/RSE-Home-Service-Robot.git
```
* Clone the below dependencies. If the turtlebot packages doesnt build delete the turtlebot packages from the catkin_ws, clone the below packages for your ros distro and try to build it. change the melodic-devel to match your distro
```
git clone https://github.com/ros-perception/slam_gmapping.git
git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
```
## Packages and Directories Structure
    catkin_ws/src/
    ├── CMakeLists.txt
    ├── my_robot
    │   ├── CMakeLists.txt
    │   ├── launch
    │   │   ├── navigation.launch 			    # contains turtlebot navigation nodes with our custom world 
    │   │   ├── slam.launch       			    # contains nodes for slam with gmapping  
    │   │   └── world.launch      			    # contains gazebo robot spawn nodes 
    │   ├── maps                  			    # contains map built with our custom world
    │   │   ├── map.pgm           
    │   │   └── map.yaml
    │   ├── package.xml
    │   ├── rviz
    │   │   └── turtlebot3_navigation.rviz  # rviz configuration files to visualise home service project
    │   ├── scripts
    │   │   ├── add_marker.py               # add_objects python node  
    │   │   ├── add_marker.sh				        # add_objects shell script 
    │   │   ├── home_service.py				      # home_service python node 
    │   │   ├── home_service.sh				      # home_service shell script 
    │   │   ├── launch.sh
    │   │   ├── pick_objects.py				      # pick_objects python node
    │   │   ├── pick_objects.sh				      # pick_objects shell script
    │   │   ├── test_navigation.sh			    # test_navigation shell script
    │   │   └── test_slam.sh				        # test_slam shell script
    │   └── worlds
    │       └── myworld.world				        # custom gazebo world built with building editor
    ├── slam_gmapping                       # gmapping ros package
    ├── turtlebot3							            # turtlebot ros packages	
    │   ├── turtlebot3
    │   ├── turtlebot3_bringup
    │   ├── turtlebot3_description
    │   ├── turtlebot3_example
    │   ├── turtlebot3_navigation
    │   ├── turtlebot3_slam
    │   └── turtlebot3_teleop
    ├── turtlebot3_msgs						          # turtlebot ros packages
    └── turtlebot3_simulations				      # turtlebot ros packages
    
* Navigate to catkin_ws workspace and build it.
```
cd ~/catkin_ws/ && catkin_make
```
* Source the workspace. 
```
source devel/setup.bash
```
* If the turtlebot packags doesnt build delete the turtlebot packages from the catkin_ws, clone the below packages for your ros distro and try to build it. change the melodic-devel to match your distro
```
git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
```
* Turn python and shell scripts to executable.
```
chmod +x catkin_ws/src/my_robot/scripts/*
```
* Navigate to scripts directory and run the scripts.
* to test slam run below script and use teleop terminal to move the robot around the world to generate map.
```
./test_slam.sh
```
* once you complete generating total environment save the map with the below command in a separate terminal.
```
rosrun map_server map_saver -f catkin_ws/src/my_robot/maps/map
```
* to test navigation run below script and use 2D Nav goal in Rviz to check go to goal function.
```
./test_navigation.sh
```
* to send multiple goals for the robot to reach use the below script.
```
./pick_objects.sh
```
* to publish a marker to rviz use the below script.
```
./add_marker.sh
```
* to simulate a home service robot that picks and drops object between places use the below script.
```
./home_service.sh
```
## Great we have successfully simulated a home service robot that picks and drops object between places
