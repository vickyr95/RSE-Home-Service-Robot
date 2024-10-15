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
  

  
