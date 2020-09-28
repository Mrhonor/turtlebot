#include "aruco_listener.h"

#include <image_transport/image_transport.h>

// opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <thread>

#include "aruco_listener_subscriber.h"
#include "aruco_listener_publisher.h"


#define PI 3.1415926535
#define LU 0
#define RU 1
#define LD 2
#define RD 3
#define X  0
#define Y  1
#define Z  2

aruco_listener::aruco_listener(ros::NodeHandle &n)
{
	ROS_INFO("init");
	count = 0;
	IsGetTarget = false;
	Subscriber = new aruco_listener_subscriber();
	Subscriber->Subscriber(n, this);

	Publisher = new aruco_listener_publisher(n, this);

	std::thread thread(&aruco_listener::aruco_process, this);
	thread.detach();
	
}

void aruco_listener::aruco_process()
{
	std::unique_lock<std::mutex> lck(aruco_process_lock, std::defer_lock);
	
	while(ros::ok()){
		lck.lock();

		if(IsGetTarget){
			IsGetTarget == false;

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

			if(aruco_img_ptr != nullptr){
				cv::line(aruco_img_ptr->image, point_cv[LU], point_cv[RU], cv::Scalar(0, 0, 255));
				cv::line(aruco_img_ptr->image, point_cv[RU], point_cv[LD], cv::Scalar(0, 0, 255));
				cv::line(aruco_img_ptr->image, point_cv[LD], point_cv[RD], cv::Scalar(0, 0, 255));
				cv::line(aruco_img_ptr->image, point_cv[RD], point_cv[LU], cv::Scalar(0, 0, 255));
				cv::line(aruco_img_ptr->image, point_cv[RU], point_cv[RD], cv::Scalar(0, 0, 255));
				cv::line(aruco_img_ptr->image, point_cv[LD], point_cv[LU], cv::Scalar(0, 0, 255));
			}
		}
		else
		{
			t(0, 0) = 1;
			t(1, 0) = 0;

			LinearV = Eigen::Vector3d::Zero();
			AngularW = = Eigen::Vector3d::Zero();
		}

		lck.unlock();
		Publisher->PublishAll();
		ros::Duration(0.01).sleep();
	}

}

aruco_listener::~aruco_listener()
{
	delete Subscriber;
	delete Publisher;
}
