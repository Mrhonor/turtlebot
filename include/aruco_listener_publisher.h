#ifndef __PUBLISHER__
#define __PUBLISHER__

#include "ros/ros.h"
#include "nav_msgs/Path.h"

class aruco_listener_core;

class aruco_listener_publisher
{
private:
    aruco_listener_core* Subject;

    ros::Publisher pub_result;
    ros::Publisher pub_turtlebot_move;
    ros::Publisher pub_RobotInfo;
    ros::Publisher pub_Waiting;
    ros::Publisher pub_trajectory;

    nav_msgs::Path path_msg;
public:
    aruco_listener_publisher(ros::NodeHandle &n, aruco_listener_core* Subject_);
    
    void PublishAll();

    void PublishWaitingInfo(double x, double y, bool status);
    void PublishTrajectory(double x, double y);
};




#endif