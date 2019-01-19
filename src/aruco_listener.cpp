#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"


void aruco_poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  ROS_INFO("Orientation : w = %lf , x = %lf , y = %lf , z = %lf", msg->orientation.w, 
            msg->orientation.x, msg->orientation.y, msg->orientation.z);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "aruco_listener");

  
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/aruco_single/pose", 10, aruco_poseCallback);

  ros::spin();

  return 0;
}
