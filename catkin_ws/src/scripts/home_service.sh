#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/mugen/Lab/service_robot/catkin_ws/src/world/uworld.world " &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/mugen/Lab/service_robot/catkin_ws/src/map/mymap.yaml " &
sleep 5
xterm -e " rosrun rviz rviz -d /home/mugen/Lab/service_robot/catkin_ws/src/rvizConfig/home_service.rviz " &
sleep 15
xterm -e "rosrun add_markers add_markers " &
sleep 5
xterm -e "rosrun pick_objects pick_objects"
