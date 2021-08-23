#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::NodeHandle nh;

    // Create Publisher
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // creating marker
    visualization_msgs::Marker marker;

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    //set string position
    std::string position;

    //set parameter type
    bool pickup_done=false;
    bool dropoff_done=false;

    //Sleep for 5 sec
    ros::Duration(2.0).sleep();

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 1;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;


    // Set the scale of the marker -- 1x1x1 here means 1m on a side

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

   if (pickup_done==false){

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    //published the marker
    marker_pub.publish(marker);
    ROS_INFO("Item location identified");


    //Sleep for 5 sec
    ros::Duration(5.0).sleep();
    
    //set new bool
    pickup_done=true;
   }


   if (pickup_done==true){

    // remove the first marker
    marker.action = visualization_msgs::Marker::DELETE;

    // Publish the removed marker 
    marker_pub.publish(marker);
    ROS_INFO("Item has been pickup");
          
    //Sleep for 5 sec
    ros::Duration(5.0).sleep();


    // Set the drop-off pose of the marker.
    // This is relative to the frame/time specified above
    marker.pose.position.x = -2.5;
    marker.pose.position.y = 5;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the marker action to ADD.
    marker.action = visualization_msgs::Marker::ADD;
          
    // Publish the marker at new position
    marker_pub.publish(marker);
    ROS_INFO("Item has been sent");
	      
    //Sleep for 5 sec
    ros::Duration(5.0).sleep();

   }

      
 
 ros::Duration(30.0).sleep();
 ros::shutdown();
  

}
