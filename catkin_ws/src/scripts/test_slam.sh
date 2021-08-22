#!/bin/sh

xterm -e " cd $(pwd)/../..;
source devel/setup.bash;
export ROBOT_INITIAL_POSE='-x -5.256984 -y -4.485685 -z 0 -R 0 -P 0 -Y 0';
roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=$(pwd)/../my_robot/worlds/myhomeoffice.world " &

sleep 5

#rosparam set /slam_gmapping/resampleThreshold 1.0;
#rosparam set /slam_gmapping/maxUrange 80;
#rosparam set /slam_gmapping/sigma 0.05;
#rosparam set /slam_gmapping/kernelSize 0.05
#rosparam set /slam_gmapping/astep 0.05
#this is for setting parameter for gmapping
xterm  -e  " cd $(pwd)/../..;
source devel/setup.bash;
rosparam set /slam_gmapping/map_update_interval 0.08;

rosparam set /slam_gmapping/linearUpdate 0.05;
rosparam set /slam_gmapping/angularUpdate 0.05;

rosparam set /slam_gmapping/srr 0.02;
rosparam set /slam_gmapping/srt 0.02;
rosparam set /slam_gmapping/str 0.02;
rosparam set /slam_gmapping/stt 0.02;

rosparam set /slam_gmapping/particles 200;

rosparam set /slam_gmapping/xmin -15.0;
rosparam set /slam_gmapping/ymin -15.0;
rosparam set /slam_gmapping/xmax 15.0;
rosparam set /slam_gmapping/ymax 15.0;
rosparam set /slam_gmapping/delta 0.01;

rosparam set /slam_gmapping/minimumScore 100.0;

roslaunch turtlebot_gazebo gmapping_demo.launch " & 

sleep 5

xterm  -e  " cd $(pwd)/../..;
source devel/setup.bash; 
roslaunch turtlebot_rviz_launchers view_navigation.launch " &

sleep 5

xterm  -e  " cd $(pwd)/../..;
source devel/setup.bash; 
roslaunch turtlebot_teleop keyboard_teleop.launch "


