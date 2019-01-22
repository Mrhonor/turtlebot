#ifndef _ARUCO_LISTENER_H_
#define _ARUCO_LISTENER_H_

// ros core
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
// msg
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
// opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#define PI 3.1415926535
#define LU 0
#define RU 1
#define LD 2
#define RD 3
#define X  0
#define Y  1
#define Z  2

using namespace std;


class aruco_listener
{
    private:

        double point[4][3];

		double fx;
		double fy;
		double cx;
		double cy;

        int count;

		cv_bridge::CvImagePtr aruco_img_ptr;

		ros::Subscriber sub_pose;  
		ros::Subscriber sub_image; 
		ros::Subscriber sub_info;  
		ros::Publisher pub_result;



    Eigen::Vector3d    t;
	

	Eigen::Quaterniond q;
	Eigen::Vector3d    v1;
	Eigen::Vector3d    v2;
	Eigen::Vector3d    v3;
	Eigen::Vector3d    v4;

    public:
        aruco_listener(ros::NodeHandle &);
        ~aruco_listener();
        void aruco_poseCallback (const geometry_msgs::PoseStamped::ConstPtr& msg);
        void aruco_imageCallback(const sensor_msgs::Image::ConstPtr& msg);
		void aruco_camera_infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
        void aruco_process();
        
};

void aruco_listener::aruco_poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

	if( ++count % 4 != 0) return;
	ROS_INFO("WELCOME");
	double size          = 0.25;   // 0.25m


	double orientation_x = msg->pose.orientation.x;
	double orientation_y = msg->pose.orientation.y;
	double orientation_z = msg->pose.orientation.z;
	double orientation_w = msg->pose.orientation.w;
<<<<<<< HEAD

    t = Eigen::Vector3d(msg->pose.position.x , msg->pose.position.y , msg->pose.position.z);
	

	q = Eigen::Quaterniond(orientation_w, orientation_x, orientation_y, orientation_z);
	v1 = Eigen::Vector3d( + size/2, + size/2, 0);
	v2 = Eigen::Vector3d( - size/2, + size/2, 0);
	v3 = Eigen::Vector3d( + size/2, - size/2, 0);
	v4 = Eigen::Vector3d( - size/2, - size/2, 0);

    v1 = q * v1 + t;
    v2 = q * v2 + t;
    v3 = q * v3 + t;
    v4 = q * v4 + t;


	ROS_INFO("Orientation : w = %lf , x = %lf , y = %lf , z = %lf", orientation_w, orientation_x, orientation_y, orientation_z); 
	ROS_INFO("position :  x = %lf , y = %lf , z = %lf", t(0, 0), t(1, 0), t(2, 0));

	double camera_pitch = atan(t(1, 0) / t(2, 0)) / PI * 180;
=======
	double position_x    = msg->pose.position.x;
	double position_y    =-msg->pose.position.y;
	double position_z    = msg->pose.position.z;
	double size          = 0.25;   // 0.25m

	ROS_INFO("Orientation : w = %lf , x = %lf , y = %lf , z = %lf", orientation_w, orientation_x, orientation_y, orientation_z); 
	ROS_INFO("position :  x = %lf , y = %lf , z = %lf", position_x, position_y, position_z);

	double camera_pitch = atan(position_y / position_z) / PI * 180;
>>>>>>> a0376e8bb45172c4686e26126a7b9ff1a9b3ec42

	double roll  = atan2(2*(orientation_w*orientation_x + orientation_y*orientation_z), 1 - 2*(orientation_x*orientation_x + 									orientation_y*orientation_y)) / PI * 180;
	double yaw   = asin(2*(orientation_w*orientation_y - orientation_x*orientation_z)) / PI * 180;
	double pitch = atan2(2*(orientation_w*orientation_z + orientation_y*orientation_x), 1 - 2*(orientation_z*orientation_z + 									orientation_y*orientation_y)) / PI * 180;

	roll = (roll > 0) ? roll - 180 : roll + 180;
<<<<<<< HEAD
/*
	point[LU][X] =cos(pitch / 180) * ((cos(roll / 180) * cos(yaw / 180) * size / 2   + position_x;
	point[LU][Y] = size/2 * pow(2, 0.5) * sin((45 + pitch) / 180) * cos(roll/ 180)  + position_y;
	point[LU][Z] = size/2 * sin(roll / 180) * sin(yaw / 180)                        + position_z;
 
	point[RU][X] = size/2 * pow(2, 0.5) * cos((45 - pitch) / 180) * cos(yaw / 180)  + position_x;
	point[RU][Y] = size/2 * pow(2, 0.5) * sin((45 - pitch) / 180) * cos(roll/ 180)  + position_y;
	point[RU][Z] = size/2 * sin(roll / 180) * sin(-yaw / 180)                       + position_z;
	
	point[LD][X] =-size/2 * pow(2, 0.5) * cos((45 + pitch) / 180) * cos(yaw / 180)  + position_x;
	point[LD][Y] =-size/2 * pow(2, 0.5) * sin((45 + pitch) / 180) * cos(roll/ 180)  + position_y;
	point[LD][Z] = size/2 * sin(-roll / 180) * sin(yaw / 180)                       + position_z;

	point[RD][X] = size/2 * pow(2, 0.5) * cos((45 - pitch) / 180) * cos(yaw / 180)  + position_x;
	point[RD][Y] =-size/2 * pow(2, 0.5) * sin((45 - pitch) / 180) * cos(roll/ 180)  + position_y;
	point[LU][Z] = size/2 * sin(-roll / 180) * sin(-yaw / 180)                      + position_z;	
*/


	ROS_INFO("roll = %lf , yaw = %lf , pitch = %lf, camera_pitch = %lf", roll, yaw, pitch,  camera_pitch); 

/*
=======

	point[LU][X] =-size/2 * pow(2, 0.5) * cos((45 + pitch) / 180) * cos(yaw / 180)  + position_x;
	point[LU][Y] = size/2 * pow(2, 0.5) * sin((45 + pitch) / 180) * cos(roll/ 180) + position_y;
	
	point[RU][X] = size/2 * pow(2, 0.5) * cos((45 - pitch) / 180) * cos(yaw / 180)  + position_x;
	point[RU][Y] = size/2 * pow(2, 0.5) * sin((45 - pitch) / 180) * cos(roll/ 180) + position_y;
	
	point[LD][X] =-size/2 * pow(2, 0.5) * cos((45 + pitch) / 180) * cos(yaw / 180)  + position_x;
	point[LD][Y] =-size/2 * pow(2, 0.5) * sin((45 + pitch) / 180) * cos(roll/ 180) + position_y;

	point[RD][X] = size/2 * pow(2, 0.5) * cos((45 - pitch) / 180) * cos(yaw / 180)  + position_x;
	point[RD][Y] =-size/2 * pow(2, 0.5) * sin((45 - pitch) / 180) * cos(roll/ 180) + position_y;
	
	distance     = position_z;

	ROS_INFO("roll = %lf , yaw = %lf , pitch = %lf, camera_pitch = %lf", roll, yaw, pitch,  camera_pitch); 

>>>>>>> a0376e8bb45172c4686e26126a7b9ff1a9b3ec42
    ROS_INFO("point[LU][X] = %lf, point[LU][Y] = %lf ", point[LU][X], point[LU][Y]);
    ROS_INFO("point[RU][X] = %lf, point[RU][Y] = %lf ", point[RU][X], point[RU][Y]);
    ROS_INFO("point[LD][X] = %lf, point[LD][Y] = %lf ", point[LD][X], point[LD][Y]);
    ROS_INFO("point[RD][X] = %lf, point[RD][Y] = %lf ", point[RD][X], point[RD][Y]); 
<<<<<<< HEAD
*/

	
=======

	aruco_process();
>>>>>>> a0376e8bb45172c4686e26126a7b9ff1a9b3ec42
}

void aruco_listener::aruco_imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{

	try {
		aruco_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
<<<<<<< HEAD
		aruco_process();
		
=======
>>>>>>> a0376e8bb45172c4686e26126a7b9ff1a9b3ec42
	}
	catch(...)
	{
		ROS_ERROR_STREAM("callback error");
	}
}

void aruco_listener::aruco_camera_infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
	fx = msg->K[0];
	cx = msg->K[2];
	fy = msg->K[4];
	cy = msg->K[5];
}

aruco_listener::aruco_listener(ros::NodeHandle &n)
{
	ROS_INFO("init");
<<<<<<< HEAD
	count = 0;
=======
	
>>>>>>> a0376e8bb45172c4686e26126a7b9ff1a9b3ec42
	sub_pose  = n.subscribe("/aruco_single/pose"       , 10, &aruco_listener::aruco_poseCallback       , this);
	sub_image = n.subscribe("/camera/color/image_raw"  , 10, &aruco_listener::aruco_imageCallback      , this);
	sub_info  = n.subscribe("/camera/color/camera_info", 10, &aruco_listener::aruco_camera_infoCallback, this);

	pub_result = n.advertise<sensor_msgs::Image>("/lion_eyes/result", 10);
}

void aruco_listener::aruco_process()
{
<<<<<<< HEAD
	cv::Point2d point_cv[5];

	point_cv[LU].x = v1(0, 0) * fx / v1(2, 0) + cx;
	point_cv[LU].y = v1(1, 0) * fy / v1(2, 0) + cy;
	
	point_cv[RU].x = v2(0, 0) * fx / v2(2, 0) + cx;
    point_cv[RU].y = v2(1, 0) * fy / v2(2, 0) + cy;
	
	point_cv[RD].x = v3(0, 0) * fx / v3(2, 0) + cx;
    point_cv[RD].y = v3(1, 0) * fy / v3(2, 0) + cy;

	point_cv[LD].x = v4(0, 0) * fx / v4(2, 0) + cx;
    point_cv[LD].y = v4(1, 0) * fy / v4(2, 0) + cy;

	point_cv[4].x = t(0, 0) * fx / t(2, 0) + cx;
	point_cv[4].y = t(1, 0) * fy / t(2, 0) + cy;
/*
    ROS_WARN_STREAM("point_cv[LU][X] = " << point_cv[LU].x <<", point_cv[LU][Y] =  "<< point_cv[LU].y);
    ROS_WARN_STREAM("point_cv[RU][X] = " << point_cv[RU].x <<", point_cv[RU][Y] =  "<< point_cv[RU].y);
    ROS_WARN_STREAM("point_cv[LD][X] = " << point_cv[LD].x <<", point_cv[LD][Y] =  "<< point_cv[LD].y);
    ROS_WARN_STREAM("point_cv[RD][X] = " << point_cv[RD].x <<", point_cv[RD][Y] =  "<< point_cv[RD].y);
*/
	try {
        //cv::circle(aruco_img_ptr->image, point_cv[4], 100 / t(2, 0),cv::Scalar(0, 255, 0));

		cv::line(aruco_img_ptr->image, point_cv[LU], point_cv[RU], cv::Scalar(0, 0, 255));
	  	cv::line(aruco_img_ptr->image, point_cv[RU], point_cv[LD], cv::Scalar(0, 0, 255));
		cv::line(aruco_img_ptr->image, point_cv[LD], point_cv[RD], cv::Scalar(0, 0, 255));
		cv::line(aruco_img_ptr->image, point_cv[RD], point_cv[LU], cv::Scalar(0, 0, 255));
		cv::line(aruco_img_ptr->image, point_cv[RU], point_cv[RD], cv::Scalar(0, 0, 255));
		cv::line(aruco_img_ptr->image, point_cv[LD], point_cv[LU], cv::Scalar(0, 0, 255));
=======
	cv::Point2d point_cv[4];
	point_cv[LU].x = point[LU][X] * fx / distance + cx;
	point_cv[LU].y = point[LU][Y] * fy / distance + cy;
	point_cv[RU].x = point[RU][X] * fx / distance + cx;
	point_cv[RU].y = point[RU][Y] * fy / distance + cy;
	point_cv[LD].x = point[LD][X] * fx / distance + cx;
	point_cv[LD].y = point[LD][Y] * fy / distance + cy;
	point_cv[RD].x = point[RD][X] * fx / distance + cx;
	point_cv[RD].y = point[RD][Y] * fy / distance + cy;

	try {
		cv::line(aruco_img_ptr->image, point_cv[0], point_cv[1], cv::Scalar(0, 0, 255));
		cv::line(aruco_img_ptr->image, point_cv[1], point_cv[2], cv::Scalar(0, 0, 255));
		cv::line(aruco_img_ptr->image, point_cv[2], point_cv[3], cv::Scalar(0, 0, 255));
		cv::line(aruco_img_ptr->image, point_cv[3], point_cv[0], cv::Scalar(0, 0, 255));
>>>>>>> a0376e8bb45172c4686e26126a7b9ff1a9b3ec42

		pub_result.publish(aruco_img_ptr->toImageMsg());
	}
	catch(...)
	{
		ROS_ERROR_STREAM("process error");
	}
}

aruco_listener::~aruco_listener()
{

}
#endif
