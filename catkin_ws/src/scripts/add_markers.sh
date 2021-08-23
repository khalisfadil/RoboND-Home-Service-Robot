#!/bin/sh

xterm -e " cd $(pwd)/../..;

source devel/setup.bash;

export ROBOT_INITIAL_POSE='-x -5.256984 -y -4.485685 -z 0 -R 0 -P 0 -Y 0';
roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=$(pwd)/../my_robot/worlds/myhomeoffice.world" &

sleep 5

xterm  -e  " cd $(pwd)/../..;
source devel/setup.bash; 
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/../map/map.yaml" & 

sleep 10

xterm  -e  " cd $(pwd)/../..;
source devel/setup.bash; 
roslaunch turtlebot_rviz_launchers view_navigation.launch " &

sleep 15

xterm  -e  " cd $(pwd)/../..;

source devel/setup.bash; 

rosrun add_markers add_markers_2 "
