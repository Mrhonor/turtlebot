#ifndef _ARUCO_LISTENER_H_
#define _ARUCO_LISTENER_H_

// ros core
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <mutex>
#include <cstring>

using namespace std;

class aruco_listener_subscriber;
class aruco_listener_publisher;

class aruco_listener
{
public:
	string RobotName;

	double point[4][3];

	double fx;
	double fy;
	double cx;
	double cy;

	int count;

	bool IsGetTarget;

	cv_bridge::CvImagePtr aruco_img_ptr;

	mutex aruco_process_lock;

    Eigen::Vector3d    t;

	Eigen::Quaterniond q;
	Eigen::Vector3d    v1;
	Eigen::Vector3d    v2;
	Eigen::Vector3d    v3;
	Eigen::Vector3d    v4;

	Eigen::Vector3d    LinearV;
	Eigen::Vector3d    AngularW;

	aruco_listener_subscriber* Subscriber;
	aruco_listener_publisher*  Publisher;

public:
	aruco_listener(ros::NodeHandle &);
	~aruco_listener();
	void aruco_process();

protected:
	void GetTargetProcess();
    void DefaultProcess();
};

#endif
