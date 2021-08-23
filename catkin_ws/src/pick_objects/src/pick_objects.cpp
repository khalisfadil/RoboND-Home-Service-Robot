#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//define successful action
bool pickup_done=false;
bool dropoff_done= false;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  ros::Rate r(10);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pickup_goal;
  move_base_msgs::MoveBaseGoal dropoff_goal;

  // set up the frame parameters
  pickup_goal.target_pose.header.frame_id = "map";
  pickup_goal.target_pose.header.stamp = ros::Time::now();

  dropoff_goal.target_pose.header.frame_id = "map";
  dropoff_goal.target_pose.header.stamp = ros::Time::now();

  // Define a positions and orientations for the robot to reach
  pickup_goal.target_pose.pose.position.x = 1.0;
  pickup_goal.target_pose.pose.position.y = 0.0;
  pickup_goal.target_pose.pose.orientation.w = 1.0;

  dropoff_goal.target_pose.pose.position.x = -2.5;
  dropoff_goal.target_pose.pose.position.y = 5.0;
  dropoff_goal.target_pose.pose.orientation.w = 1.0;

  //setting parameters
  n.setParam("/position", "start_position");

  // Send Pick Up Goal
  ROS_INFO("Sending pick up goal");
  ac.sendGoal(pickup_goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Succesfully arrive the pick up location");

    //pickup done
    pickup_done=true;

    //update setting parameters for pick position
    n.setParam("/position", "pickup_position");

  }

  else{

    //pickup fail
    pickup_done=false;
    ROS_INFO("Location for pick up cannot be reached");

    return 0;
  }

  // Wait 5 Sec
  ros::Duration(5.0).sleep();

  // Send Drop Off Goal
  ROS_INFO("Sending drop off goal");
  ac.sendGoal(dropoff_goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Succesfully arrive the drop of location");

    //dropp off done
    dropoff_done=true;

    //update setting parameters for drop off position
    n.setParam("/position", "dropoff_position");
  }

  else{
    ROS_INFO("Location for drop off cannot be reached");

    //dropp off failed
    dropoff_done=false;

    return 0;
  }

  // Wait 10 Sec
  ros::Duration(10.0).sleep();
  


  r.sleep();
}
