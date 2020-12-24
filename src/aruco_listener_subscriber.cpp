#include "aruco_listener_subscriber.h"

#include "aruco_listener.h"

#define PI 3.1415926535

using namespace std;

aruco_listener_subscriber::aruco_listener_subscriber(ros::NodeHandle &n, aruco_listener_core* Subject_){
	sec = -1;
	LastAcc = Eigen::Vector3d::Zero();
	
    Subject = Subject_;
    if(Subject != nullptr){
		string topicName = string("/") + Subject->RobotName + string("/mobile_base/sensors/imu_data");
		sub_imu     = n.subscribe(topicName, 10, &aruco_listener_subscriber::aruco_imuCallback, this);
   
		sub_gaze    = n.subscribe("/gazebo/model_states", 10, &aruco_listener_subscriber::aruco_gazeCallback, this);
		sub_control = n.subscribe("/robot_controller/control", 10, &aruco_listener_subscriber::aruco_controlCallback, this);
		sub_info    = n.subscribe("/aruco_listener/robot/info", 10, &aruco_listener_subscriber::aruco_robotInfoCallback, this);
		sub_wait    = n.subscribe("/aruco_listener/robot/wait", 10, &aruco_listener_subscriber::aruco_waitInfoCallback, this);
    }
}




void aruco_listener_subscriber::aruco_imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
	// unique_lock<mutex> lck = unique_lock<mutex>(Subject->aruco_process_lock, defer_lock);
	// lck.lock();
	// Eigen::Vector3d coordinate = Subject->CurrentCoordinate;
	// Eigen::Vector3d v = Subject->CurrentLinearV;
	// lck.unlock();
	

	// if(sec != -1)
	// {
	// 	double DeltaTime = msg->header.stamp.sec -sec + (msg->header.stamp.nsec * 0.0000001 - nsec * 0.0000001) * 0.01;

	// 	coordinate += v * DeltaTime;
	// 	v += LastAcc * DeltaTime;
	// }
	
	// double orientation_x = msg->orientation.x;
	// double orientation_y = msg->orientation.y;
	// double orientation_z = msg->orientation.z;
	// double orientation_w = msg->orientation.w;
	// double yaw   = asin(2*(orientation_w*orientation_y - orientation_x*orientation_z)) / PI * 180;

	// lck.lock();
	// Subject->CurrentCoordinate = coordinate;
	// Subject->CurrentLinearV = v;
	// Subject->Yaw = yaw;


	// sec = msg->header.stamp.sec;
	// nsec = msg->header.stamp.nsec;
	// LastAcc = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
}

void aruco_listener_subscriber::aruco_gazeCallback(const gazebo_msgs::ModelStates::ConstPtr& msg){
	unique_lock<mutex> lck = unique_lock<mutex>(Subject->aruco_process_lock, defer_lock);
	int id = 0;
	auto RobotName_it = end(Subject->RobotName);
	RobotName_it--;
	for(auto i : msg->name){
		auto end_it = end(i);
		end_it--;
		if(*RobotName_it == *end_it){
			break;
		}
		id++;
	}
	

	double orient_x = msg->pose[id].orientation.x;
	double orient_y = msg->pose[id].orientation.y;
	double orient_z = msg->pose[id].orientation.z;
	double orient_w = msg->pose[id].orientation.w;
	double yaw = atan2(2 * (orient_x*orient_y + orient_w*orient_z), orient_w*orient_w + orient_x*orient_x - orient_y*orient_y - orient_z*orient_z) / PI * 180;
	
	Eigen::Vector3d(msg->pose[id].position.x, msg->pose[id].position.y, msg->pose[id].position.z);

	lck.lock();
	Subject->CurrentLinearV = Eigen::Vector3d(msg->twist[id].linear.x, msg->twist[id].linear.y, msg->twist[id].linear.z);
	Subject->CurrentCoordinate = Eigen::Vector3d(msg->pose[id].position.x, msg->pose[id].position.y, msg->pose[id].position.z);
	Subject->YawSpeed = msg->twist[id].angular.z;
	Subject->Yaw = yaw;

}

void aruco_listener_subscriber::aruco_controlCallback(const aruco_listener::aruco_msg::ConstPtr& msg){
	if(Subject->RobotName == msg->robotName){
		unique_lock<mutex> lck = unique_lock<mutex>(Subject->aruco_process_lock, defer_lock);
		lck.lock();
		Subject->TargetV = Eigen::Vector3d(msg->linear.x, msg->linear.y, msg->linear.z);
		Subject->TargetYaw = msg->yaw;
		Subject->IsGetTarget = true;
	}

}

void aruco_listener_subscriber::aruco_robotInfoCallback(const aruco_listener::aruco_msg::ConstPtr& msg){
    string name = msg->robotName;
	if (name == Subject->RobotName)
	{
		return;
	}

    unique_lock<mutex> lck = unique_lock<mutex>(Subject->aruco_process_lock, defer_lock);
    lck.lock();
    for(auto &i : Subject->Robots){
        if(i.Name == name){
            i.v = msg->linear.x;
            i.yaw = msg->yaw;
            i.x = msg->x;
            i.y = msg->y;
            return;
        }
    }
    RobotInfo NewInfo;
    NewInfo.Name = name;
    NewInfo.v = msg->linear.x;
    NewInfo.yaw = msg->yaw;
    NewInfo.x = msg->x;
    NewInfo.y = msg->y;
    Subject->Robots.push_back(NewInfo);

}

void aruco_listener_subscriber::aruco_waitInfoCallback(const aruco_listener::aruco_msg::ConstPtr& msg){
	string name = msg->robotName;
	if (name == Subject->RobotName)
	{
		return;
	}

	unique_lock<mutex> lck = unique_lock<mutex>(Subject->aruco_process_lock, defer_lock);
	lck.lock();
    if(fabs(msg->yaw) <= 0.01){
		RobotInfo NewInfo;
		NewInfo.Name = name;
		for(auto &i : Subject->CrossRoad){
			if(i.IsWithingTuringPoints(msg->x, msg->y, i.boundaryLength)){
				i.WaitQuene.push_back(NewInfo);
			}
		}
	}
	else
	{
		for(auto &i : Subject->CrossRoad){
			if(i.IsWithingTuringPoints(msg->x, msg->y, i.boundaryLength) && name != Subject->RobotName){
				i.WaitQuene.erase(i.WaitQuene.begin());
			}
		}
	}	
}
