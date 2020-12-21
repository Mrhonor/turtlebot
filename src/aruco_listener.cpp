#include "aruco_listener.h"

#include <image_transport/image_transport.h>

// opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <thread>
#include <cmath>

#include "aruco_listener_subscriber.h"
#include "aruco_listener_publisher.h"
#include "RampFunc.h"

using namespace std;

#define PI 3.1415926535
#define LU 0
#define RU 1
#define LD 2
#define RD 3
#define X  0
#define Y  1
#define Z  2
#define MAX_SPEED 2.f
#define FURRENCY  50.f

aruco_listener_core::aruco_listener_core(ros::NodeHandle &n)
{
	ROS_INFO("init");
	count = 0;
	IsGetTarget = false;

	ros::param::get("~robot_name", RobotName);
	ros::param::get("~startX",     StartX);
	ros::param::get("~startY",     StartY);
	ROS_INFO_STREAM("robot name: " << RobotName);

	CurrentLinearV  = Eigen::Vector3d::Zero();
	CurrentCoordinate = Eigen::Vector3d::Zero();
	TargetV = Eigen::Vector3d(1,0,0);
	Yaw = 0.0;
	TargetYaw = 0.0;

	CurrentCoordinate(0, 0) = StartX;
	CurrentCoordinate(1, 0) = StartY;

	double kp_v, ki_v, kd_v, kp_yaw, ki_yaw, kd_yaw;
	ros::param::get("~kp_v", kp_v);
	ros::param::get("~ki_v", ki_v);
	ros::param::get("~kd_v", kd_v);
	ros::param::get("~kp_yaw", kp_yaw);
	ros::param::get("~ki_yaw", ki_yaw);
	ros::param::get("~kd_yaw", kd_yaw);

	PID_V.kp = kp_v;
	PID_V.ki = ki_v;
	PID_V.kd = kd_v;
	PID_V.Ilimit = Eigen::Vector3d(0.5, 0.5, 0.5);
	PID_V.Outlimit = Eigen::Vector3d(0.8, 0.8, 0.8);

	PID_Yaw.kp = kp_yaw;
	PID_Yaw.ki = ki_yaw;
	PID_Yaw.kd = kd_yaw;
	PID_Yaw.Ilimit = 0.2;
	PID_Yaw.Outlimit = 0.7;

	Subscriber = new aruco_listener_subscriber(n, this);
	Publisher = new aruco_listener_publisher(n, this);


	thread thread(&aruco_listener_core::aruco_process, this);
	thread.detach();
	
}

void aruco_listener_core::aruco_process()
{
	unique_lock<mutex> lck(aruco_process_lock, defer_lock);
	
	while(ros::ok()){
		lck.lock();

		if(IsGetTarget){
			IsGetTarget = false;
			GetTargetProcess();
		}
		else
		{
			DefaultProcess();
		}

		Publisher->PublishAll();

		lck.unlock();
		ros::Duration(1 / FURRENCY).sleep();
	}
}

aruco_listener_core::~aruco_listener_core()
{
	delete Subscriber;
	delete Publisher;
}

void aruco_listener_core::GetTargetProcess(){
	// LinearV = PID_V.pid_calc(TargetV, CurrentLinearV);
	LinearV = TargetV;
	AngularW(2, 0) = PID_Yaw.pid_calc(TargetYaw, Yaw);
}

void aruco_listener_core::DefaultProcess(){

	LinearV = RampFunc<Eigen::Vector3d>()(LinearV, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.03, 0.03, 0.03));
	AngularW(2, 0) = RampFunc<double>()(AngularW(2, 0), 0, 0.03);
}

