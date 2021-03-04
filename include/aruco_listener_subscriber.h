#ifndef __SUBSCRIBER__
#define __SUBSCRIBER__

#include "ros/ros.h"
// msg
#include "geometry_msgs/PoseStamped.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "aruco_listener/aruco_msg.h"
// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
// cpp
#include <mutex>


class aruco_listener_core;

class aruco_listener_subscriber
{
private:
    aruco_listener_core* Subject;

    ros::Subscriber sub_imu;
    ros::Subscriber sub_gaze;
    ros::Subscriber sub_control;
    ros::Subscriber sub_info;
    ros::Subscriber sub_wait;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_ar;


    int sec;
    int nsec;
    
    Eigen::Vector3d LastAcc;

public:
    aruco_listener_subscriber(){}
    aruco_listener_subscriber(ros::NodeHandle &n, aruco_listener_core* Subject_);

private:
    void aruco_imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void aruco_gazeCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void aruco_controlCallback(const aruco_listener::aruco_msg::ConstPtr& msg);
    void aruco_robotInfoCallback(const aruco_listener::aruco_msg::ConstPtr& msg);
    void aruco_waitInfoCallback(const aruco_listener::aruco_msg::ConstPtr& msg);
    void aruco_odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void aruco_arCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
};

#endif