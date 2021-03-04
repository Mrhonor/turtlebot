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
	IsPositionControl = false;

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

	double kp_q, ki_q, kd_q, kp_v, ki_v, kd_v, kp_yaw, ki_yaw, kd_yaw;
	ros::param::get("~kp_q", kp_q);
	ros::param::get("~ki_q", ki_q);
	ros::param::get("~kd_q", kd_q);
	ros::param::get("~kp_v", kp_v);
	ros::param::get("~ki_v", ki_v);
	ros::param::get("~kd_v", kd_v);
	ros::param::get("~kp_yaw", kp_yaw);
	ros::param::get("~ki_yaw", ki_yaw);
	ros::param::get("~kd_yaw", kd_yaw);

	PID_Q.kp = kp_q;
	PID_Q.ki = ki_q;
	PID_Q.kd = kd_q;
	PID_Q.Ilimit = Eigen::Vector3d(0.3, 0.3, 0.3);
	PID_Q.Outlimit = Eigen::Vector3d(0.9,0.9, 0.9);

	PID_V.kp = kp_v;
	PID_V.ki = ki_v;
	PID_V.kd = kd_v;
	PID_V.Ilimit = Eigen::Vector3d(0.5, 0.5, 0.5);
	PID_V.Outlimit = Eigen::Vector3d(0.8, 0.8, 0.8);

	PID_Yaw.kp = kp_yaw;
	PID_Yaw.ki = ki_yaw;
	PID_Yaw.kd = kd_yaw;
	PID_Yaw.Ilimit = 0.1;
	PID_Yaw.Outlimit = 0.6;

	TurningPoint p[1];
	p[0].x = 2.0;
	p[0].y = 0;
	// p[0].x = 5;
    // p[0].y = 0;
	// p[1].x = 6.55;
	// p[1].y = 4.76;
	// p[2].x = 8.09;
	// p[2].y = 0;
	// p[3].x = 13.09;
	// p[3].y = 0;
	// p[4].x = 9.05;
	// p[4].y = -2.94;
 	// p[5].x = 10.59;
	// p[5].y = -7.70;
 	// p[6].x = 6.55;
	// p[6].y = -4.76;
 	// p[7].x = 2.5;
	// p[7].y = -7.7;
 	// p[8].x = 4.05;
	// p[8].y = -2.94;
 	// p[9].x = 0;
	// p[9].y = 0;

 	for(int i = 0; i < 1; i++) {
		p[i].boundaryLength = 0.1;
		CrossRoad.push_back(p[i]);
	}

	CurrentTurningPoint = nullptr;

	SetDistination(CrossRoad[count]);

	Subscriber = new aruco_listener_subscriber(n, this);
	Publisher = new aruco_listener_publisher(n, this);


	thread thread(&aruco_listener_core::aruco_process, this);
	thread.detach();
	
}

void aruco_listener_core::aruco_process()
{
	unique_lock<mutex> lck(aruco_process_lock, defer_lock);
	ros::Duration(5).sleep();
	while(ros::ok()){
		lck.lock();
		
		// policy choosen
		OnSelfControl();
		
		// pid control
		GetTargetProcess();

		if(IsGetTarget){
			IsGetTarget = false;
			Publisher->PublishTrajectory(CurrentCoordinate(0, 0), CurrentCoordinate(1, 0));
		}
		
		// publish
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
	if(IsPositionControl){
		TargetV = PID_Q.pid_calc(Distination, CurrentCoordinate);
		TargetV(0, 0) = fabsf(TargetV(0,0));
	}

	LinearV = RampFunc<Eigen::Vector3d>()(LinearV, TargetV, Eigen::Vector3d(0.03, 0.03, 0.03));
	AngularW(2, 0) = PID_Yaw.pid_calc(TargetYaw, Yaw);
}

void aruco_listener_core::DefaultProcess(){
	LinearV = RampFunc<Eigen::Vector3d>()(LinearV, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.03, 0.03, 0.03));
	AngularW(2, 0) = RampFunc<double>()(AngularW(2, 0), 0, 0.03);
}

void aruco_listener_core::OnSelfControl(){
    if(fabs(CurrentCoordinate(0,0) - Distination(0,0)) <= 0.1 && fabs(CurrentCoordinate(1,0) - Distination(1,0)) <= 0.1){
		TargetV(0,0) = 0;
		IsPositionControl = false;
		TargetYaw = 0;
		return;
    }
	
    bool CanPassTheCrossRoad = true;
	if(CurrentTurningPoint == nullptr){
		for(auto &p : CrossRoad){
			// check is it withing the turing points
			if(p.IsWithingTuringPoints(CurrentCoordinate(0,0), CurrentCoordinate(1,0), p.boundaryLength)){
				CurrentTurningPoint = &p;
				if (p.WaitQuene.size() != 0)
				{
					CanPassTheCrossRoad = false;
				}
				RobotInfo newInfo;
				newInfo.Name = RobotName;
				p.WaitQuene.push_back(newInfo);
				Publisher->PublishWaitingInfo(p.x, p.y, true);
			}
		}
	}
	else
	{
		// check it should wait or go
		if(CurrentTurningPoint->WaitQuene[0].Name == RobotName){
			if(!CurrentTurningPoint->IsWithingTuringPoints(CurrentCoordinate(0,0), CurrentCoordinate(1,0), CurrentTurningPoint->boundaryLength + 0.1)){
				Publisher->PublishWaitingInfo(CurrentTurningPoint->x, CurrentTurningPoint->y, false);
				CurrentTurningPoint->WaitQuene.erase(CurrentTurningPoint->WaitQuene.begin());
				CurrentTurningPoint = nullptr;
			}
		}
		else
		{
			CanPassTheCrossRoad = false;
		}
		
	}
	
    TargetYaw = atan2(Distination(1,0) - CurrentCoordinate(1,0), Distination(0,0) - CurrentCoordinate(0,0)) / PI * 180.0;
	
    if(fabs(TargetYaw - Yaw) < 20 && CanPassTheCrossRoad){
        //search others to follow
        double MinDistance = -1;
        for(auto &i : Robots){
            if(abs(i.yaw - Yaw) < 30){
                double Distance = (i.x - CurrentCoordinate(0,0)) * cos(Yaw / 180.0 * PI) + (i.y - CurrentCoordinate(1,0)) * sin(Yaw / 180.0 * PI);  
                if(Distance > 0 && (MinDistance == -1 || Distance < MinDistance)){
                    MinDistance = Distance;
                    Following = &i;
                }
            }
        }

        if (MinDistance > 3) //catch up with the leader
        {
            TargetV(0,0) = 1.2;
			IsPositionControl = false;
        }
        else if(MinDistance == -1) // nobody to follow
        {
			// TargetV(0,0) = 0.6;
			IsPositionControl = true;
        }
        else
        {
            // following
            TargetV(0,0) = Following->v;
			IsPositionControl = false;
        }
        
    }
    else
    {
        TargetV(0,0) = 0;
		IsPositionControl = false;
    }
    
}

void aruco_listener_core::SetDistination(TurningPoint &NewTarget){
	Distination = Eigen::Vector3d(NewTarget.x, NewTarget.y, 0);
}
