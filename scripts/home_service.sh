#!/bin/sh

xterm -e "source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/src/world/house " &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/src/map/map.yaml " &
sleep 5
xterm -e " rosrun rviz rviz -d $(pwd)/src/rvizConfig/home_service.rviz " &
sleep 10
xterm -e " source devel/setup.bash; roslaunch add_markers add_markers_robot_tracking.launch " &
sleep 5
xterm -e " source devel/setup.bash; roslaunch pick_objects pick_objects.launch "
