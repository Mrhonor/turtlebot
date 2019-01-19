#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"


void aruco_poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ROS_INFO("WELCOME");
  double x = msg->pose.orientation.x;
  double y = msg->pose.orientation.y;
  double z = msg->pose.orientation.z;
  double w = msg->pose.orientation.w;
  ROS_INFO("Orientation : w = %lf , x = %lf , y = %lf , z = %lf", w, x, y, z); 
  
  double roll  = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));
  double yaw   = asin(2*(w*y - x*z));
  double pitch = - atan2(2*(w*z + y*x), 1 - 2*(z*z + y*y)); 
  ROS_INFO("roll = %lf , yaw = %lf , pitch = %lf", roll, yaw, pitch); 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_listener");
  
  ros::NodeHandle n;
  ROS_INFO("START 1");
  ros::Subscriber sub = n.subscribe("/aruco_single/pose", 10, aruco_poseCallback);
  ROS_INFO("START 2");

  ros::spin();

  return 0;
}
