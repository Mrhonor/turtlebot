#ifndef __PUBLISHER__
#define __PUBLISHER__

#include "ros/ros.h"

class aruco_listener;

class aruco_listener_publisher
{
private:
    aruco_listener* Subject;

    ros::Publisher pub_result;
    ros::Publisher pub_turtlebot_move;
    ros::NodeHandle n;

public:
    aruco_listener_publisher(ros::NodeHandle &n_, aruco_listener* Subject_);
    
    void PublishAll();

    
};




#endif