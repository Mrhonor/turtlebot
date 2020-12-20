#ifndef __SUBSCRIBER__
#define __SUBSCRIBER__

#include "ros/ros.h"
// msg
#include "geometry_msgs/PoseStamped.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Imu.h"
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

    int sec;
    int nsec;
    
    Eigen::Vector3d LastAcc;

    std::unique_lock<std::mutex> lck;

public:
    void Subscriber(ros::NodeHandle &n, aruco_listener_core* Subject_);

private:
    void aruco_imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void aruco_gazeCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
};

#endif