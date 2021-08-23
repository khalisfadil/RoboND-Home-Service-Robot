# Home Service Robot

Home Service Robot is a robotic project to create a robot that able to autonomously map an environment and navigate by itself to pick-up and deliver object in the generated environment. This project use the official ROS package named Turtlebot to map, localize and navigate in the already generated world.

## Installation

Clone the package manager [rospkg](git://github.com/ros/rospkg.git ) and install.

```bash
git clone git://github.com/ros/rospkg.git
python setup.py install --user 
```
or use apt:
```bash
sudo apt-get install python-rospkg
```
or use pip:
```bash
sudo pip install -U rospkg
```

## Project description

### Creating Shell Scripts

This project consist of multiple launch of ROS  nodes. Thus, generating a Shell scripts is really helpful in order to launch a multiple nodes within a separate Terminal by only using a single command.

### Catkin Workspace and Official ROS package

This project use several packages from official ROS packages. These packages are imported and installed under the ```src``` file. The packages is listed below:

1. [gmapping](https://github.com/ros-perception/slam_gmapping)
2. [turtlebot_teleop](https://github.com/turtlebot/turtlebot)
3. [turtlebot_rviz_launchers](https://github.com/turtlebot/turtlebot_interactions)
4. [turtlebot_gazebo](https://github.com/turtlebot/turtlebot_simulator)

### SLAM
SLAM is used in this project to generate and autonomously map the world environment that designed earlier. The world file can be found in ```src/my_robot/worlds```. Turtlebot is deploy inside the designed world. By interfacing it with SLAM package, the turtlebot will be able to vizualize and mapping the environment by using  ```rviz```. The turtlebot is being moved manually by using teleop keyboard.

![alt text](https://github.com/khalisfadil/RoboND-Home-Service-Robot/blob/main/images/slam%20mapping.png)

after completing mapping the environment, the image from the Rviz is save.

```bash
rosrun map_server map_server map.yaml
```
At this stage, it is a bit tricky due to the unpredictable size of the map. Thus, the configuration need to be edited manually in the ```map.yaml``` file. The generated world image is also edited by using adobe Photoshop to resize and crop unnecessary area. To make the launch easier, a shell script named ```test_slam.sh```is created which consist of world launch, gmapping, view navigation and teleop keyboard. 

### Localization and Navigation
The Turtlebot is deployed again in the same world, but this time, it is used to test the navigation and localization with the help of the SLAM generated map before. In this section, the ROS navigation stack is used by letting the Turtlebot to do the path planning and obstacle avoiding after receiving a goal destination command. In ```rviz```, the goal destination can be set by using ```2D nav goal``` and the turtlebot should be able to generates it's own path towards the goal destination.

![alt text](https://github.com/khalisfadil/RoboND-Home-Service-Robot/blob/main/images/test_navigation.gif)

shell script ```test_navigation.sh``` is created consist of world launch, AMCL launch and navigation launch.

### Navigation Goal node
this section will create a node in C++ to communicate with the ROS navigation stack to send an input about the goal destination. A package named `pick_objects` is created to set the pickup goal location and a dropof goal location. Turtlebot will receive an information to generate a path planning towards the goal destination. At the same time, a ROSINFO massage is generated indicating that the item has been picked up or has been delivered.

![alt text](https://github.com/khalisfadil/RoboND-Home-Service-Robot/blob/main/images/pick_objects.gif)

A shell script named `pick_object.sh` is created that consist of turtlebot launch, AMCL, rviz and the *pick_objects* node. 

### Adding virtual mark
to add the sweet to the simulation, a virtual object with markers is created in rviz. The virtual object will represent the object that need to be pickup as the the Turtlebot reached the Pickup destination and also represent the object that being delivered at the Dropof location.
the algorithm idea are:
* publish the marker at pickup location
* paused 5 seconds
* Hide the marker
* Pause 5 seconds
* Publish the marker at dropof location

A shell script named `add_markers.sh` is generated consisting of turtlebot, AMCL, rviz and *add_markers* node.

### Finalizing Home Service Robot
Lastly, by connecting the nodes *pick_objects* and *add_markers*, the robot should be able to simulate a complete simulation of receiving an input goal destination, path planning navigation, virtual marker appear, picking up the virtual marker and send to the second input goal destination. My first idea was to make the node *add_markers* subscribing to the odometry and comunicating with the *pick_objects* node through subscriber and publisher principle. However, I was not able to complete the code program as the *add_markers* node not able to read the publisher massage from the *pick_objects* nodes. 

It is easier to use a `rosparam`that allow me to store a multiple data on the ROS parameter server as `string` value. For example, in the *pick_objects* nodes, when the Turtlebot reach the first destination, the parameter data is stored by using `rosparam set` as string value named *pickup location*. Then in the second node *add_markers* will delete the marker in the map if the current Turtlebot parameter is equal to the string value *pick up location* by using `rosparam get`.

## Conclusion

This project is really fun to do with and at the same time is really challenging. As completing this project, one supposedly should be able to understand the technical requirement of creating and organizing a *catkin_file* for the development of robot software. The project also able to emphasis the use of localization, mapping, navigation and parameter setting within ROS. 
