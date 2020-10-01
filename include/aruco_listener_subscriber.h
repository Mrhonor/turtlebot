#ifndef __SUBSCRIBER__
#define __SUBSCRIBER__

#include "ros/ros.h"
// msg
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"


class aruco_listener;

class aruco_listener_subscriber
{
private:
    aruco_listener* Subject;

    ros::Subscriber sub_pose;  
    ros::Subscriber sub_image; 
    ros::Subscriber sub_info;  

public:
    void Subscriber(ros::NodeHandle &n, aruco_listener* Subject_);

private:
    void aruco_poseCallback (const geometry_msgs::PoseStamped::ConstPtr& msg);
    void aruco_imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void aruco_camera_infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

};

#endif