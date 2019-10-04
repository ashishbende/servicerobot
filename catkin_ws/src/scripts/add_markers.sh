#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/mugen/Lab/service_robot/catkin_ws/src/world/uworld.world " &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/mugen/Lab/service_robot/catkin_ws/src/map/mymap.yaml " &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 15
xterm -e "rosrun add_markers add_markers"
