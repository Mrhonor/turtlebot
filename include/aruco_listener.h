#ifndef _ARUCO_LISTENER_H_
#define _ARUCO_LISTENER_H_

// ros core
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
// cpp
#include <mutex>
#include <cstring>

#include "pid.h"
#include "Limit.h"

// using namespace std;

class aruco_listener_subscriber;
class aruco_listener_publisher;

class aruco_listener_core
{
public:
	std::string RobotName;

	double StartX;
	double StartY;
	double Yaw;
	double YawSpeed;

	Eigen::Vector3d CurrentLinearV; 	//read from Gaze
	Eigen::Vector3d CurrentCoordinate;  //read from Gaze

	Eigen::Vector3d TargetV;			
	double TargetYaw;


	int count;

	bool IsGetTarget;


	std::mutex aruco_process_lock;

	Eigen::Vector3d    LinearV;
	Eigen::Vector3d    AngularW;



private:
	aruco_listener_subscriber* Subscriber;
	aruco_listener_publisher*  Publisher;

	pid<Eigen::Vector3d, Limit<Eigen::Vector3d>> PID_V;
	pid<double, Limit<double>> PID_Yaw;

public:
	aruco_listener_core(ros::NodeHandle &);
	~aruco_listener_core();
	void aruco_process();

protected:
	void GetTargetProcess();
    void DefaultProcess();
};

#endif
