#include "aruco_listener_subscriber.h"

#include "aruco_listener.h"
// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#define PI 3.1415926535

void aruco_listener_subscriber::Subscriber(ros::NodeHandle &n, aruco_listener* Subject_){
    Subject = Subject_;
    if(Subject != nullptr){
        sub_pose  = n.subscribe("/aruco_single/pose"     , 10, &aruco_listener_subscriber::aruco_poseCallback       , this);
        sub_image = n.subscribe("/camera/rgb/image_raw"  , 10, &aruco_listener_subscriber::aruco_imageCallback      , this);
        sub_info  = n.subscribe("/camera/rgb/camera_info", 10, &aruco_listener_subscriber::aruco_camera_infoCallback, this);
    }
}


void aruco_listener_subscriber::aruco_poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	double size          = 0.25;   // 0.25m

	double orientation_x = msg->pose.orientation.x;
	double orientation_y = msg->pose.orientation.y;
	double orientation_z = msg->pose.orientation.z;
	double orientation_w = msg->pose.orientation.w;

    Subject->t = Eigen::Vector3d(msg->pose.position.x , msg->pose.position.y , msg->pose.position.z);
	

	Subject->q = Eigen::Quaterniond(orientation_w, orientation_x, orientation_y, orientation_z);
	Subject->v1 = Eigen::Vector3d(+ size/2, 0, + size/2);
	Subject->v2 = Eigen::Vector3d(- size/2, 0, + size/2);
	Subject->v3 = Eigen::Vector3d(+ size/2, 0, - size/2);
	Subject->v4 = Eigen::Vector3d(- size/2, 0, - size/2);

    Subject->v1 = Subject->q * Subject->v1 + Subject->t;
    Subject->v2 = Subject->q * Subject->v2 + Subject->t;
    Subject->v3 = Subject->q * Subject->v3 + Subject->t;
    Subject->v4 = Subject->q * Subject->v4 + Subject->t;

	double camera_pitch = atan(Subject->t(1, 0) / Subject->t(2, 0)) / PI * 180;

	double roll  = atan2(2*(orientation_w*orientation_x + orientation_y*orientation_z), 1 - 2*(orientation_x*orientation_x + orientation_y*orientation_y)) / PI * 180;
	double yaw   = asin(2*(orientation_w*orientation_y - orientation_x*orientation_z)) / PI * 180;
	double pitch = atan2(2*(orientation_w*orientation_z + orientation_y*orientation_x), 1 - 2*(orientation_z*orientation_z + orientation_y*orientation_y)) / PI * 180;

	roll = (roll > 0) ? roll - 180 : roll + 180;

	Subject->IsGetTarget = true;

	ROS_INFO("Roll: %lf, Yaw: %lf, Pitch:%lf", roll, yaw, pitch);
	
}

void aruco_listener_subscriber::aruco_imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{

	try {
		Subject->aruco_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		
	}
	catch(...)
	{
		ROS_ERROR_STREAM("callback error");
	}
}

void aruco_listener_subscriber::aruco_camera_infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
	Subject->fx = msg->K[0];
	Subject->cx = msg->K[2];
	Subject->fy = msg->K[4];
	Subject->cy = msg->K[5];
}
