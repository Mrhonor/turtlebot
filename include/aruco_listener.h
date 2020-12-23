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


typedef struct RobotInfo
{
    std::string Name;
    double x = 0;
    double y = 0;
    double v = 0;
    double yaw = 0;

    double TargetX = 5;
    double TargetY = 0;
    double ControlV = 0;
    double ControlYaw = 0;

    struct RobotInfo* Following = nullptr;
}RobotInfo;

typedef struct
{
    double x;
    double y;
    double boundaryLength;
    std::vector<RobotInfo> WaitQuene;
    bool IsWithingTuringPoints(RobotInfo* info){
        if(fabs(info->x - x) < boundaryLength && fabs(info->y - y) < boundaryLength){
            return true;
        }
        else
        {
            return false;
        }
        
    }
	bool IsWithingTuringPoints(double infoX, double infoY, double Length){
        if(fabs(infoX - x) < Length && fabs(infoY - y) < Length){
            return true;
        }
        else
        {
            return false;
        }
        
    }

    bool IsWithingWaitQuene(RobotInfo* info){
        for (auto &i : WaitQuene)
        {
            if(i.Name == info->Name){
                return true;
            }
        }
        return false;
    }
}TurningPoint;

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
	Eigen::Vector3d Distination;        //the distination of this robot
	double TargetYaw;


	int count;

	bool IsGetTarget;


	std::mutex aruco_process_lock;

	Eigen::Vector3d    LinearV;
	Eigen::Vector3d    AngularW;

	std::vector<RobotInfo> Robots;
    std::vector<TurningPoint> CrossRoad;
	

private:
	aruco_listener_subscriber* Subscriber;
	aruco_listener_publisher*  Publisher;

	pid<Eigen::Vector3d, Limit<Eigen::Vector3d>> PID_V;
	pid<double, Limit<double>> PID_Yaw;

	// it will point to a Turning Point where this robot within. If this robot isn't within any Turning Point, it will be nullptr.
	TurningPoint* CurrentTurningPoint; 
	RobotInfo* Following; // Following robot
public:
	aruco_listener_core(ros::NodeHandle &);
	~aruco_listener_core();
	void aruco_process();

private:
	void GetTargetProcess();
    void DefaultProcess();
	void OnSelfControl();
};

#endif
