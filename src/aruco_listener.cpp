#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#define PI 3.1415926535
#define LU 0
#define RU 1
#define LD 2
#define RD 3
#define X  0
#define Y  1

void aruco_poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	ROS_INFO("WELCOME");
	double orientation_x = msg->pose.orientation.x;
	double orientation_y = msg->pose.orientation.y;
	double orientation_z = msg->pose.orientation.z;
	double orientation_w = msg->pose.orientation.w;
	double position_x    = msg->pose.position.x;
	double position_y    =-msg->pose.position.y;
	double position_z    = msg->pose.position.z;
	double size          = 0.025;   // 0.025m

	ROS_INFO("Orientation : w = %lf , x = %lf , y = %lf , z = %lf", orientation_w, orientation_x, orientation_y, orientation_z); 
	ROS_INFO("position :  x = %lf , y = %lf , z = %lf", position_x, position_y, position_z);

	double camera_pitch = atan(position_y / position_z) / PI * 180;

	double roll  = atan2(2*(orientation_w*orientation_x + orientation_y*orientation_z), 1 - 2*(orientation_x*orientation_x + 									orientation_y*orientation_y)) / PI * 180;
	double yaw   = asin(2*(orientation_w*orientation_y - orientation_x*orientation_z)) / PI * 180;
	double pitch = atan2(2*(orientation_w*orientation_z + orientation_y*orientation_x), 1 - 2*(orientation_z*orientation_z + 									orientation_y*orientation_y)) / PI * 180;

	roll = (roll > 0) ? roll - 180 : roll + 180;

	double point[4][2] = {0};
	point[LU][X] =-size/2 * pow(2, 0.5) * cos((45 + pitch) / 180) * cos(yaw / 180)  + position_x;
	point[LU][Y] = size/2 * pow(2, 0.5) * sin((45 + pitch) / 180) * cos(roll/ 180) + position_y;
	
	point[RU][X] = size/2 * pow(2, 0.5) * cos((45 - pitch) / 180) * cos(yaw / 180)  + position_x;
	point[RU][Y] = size/2 * pow(2, 0.5) * sin((45 - pitch) / 180) * cos(roll/ 180) + position_y;
	
	point[LD][X] =-size/2 * pow(2, 0.5) * cos((45 + pitch) / 180) * cos(yaw / 180)  + position_x;
	point[LD][Y] =-size/2 * pow(2, 0.5) * sin((45 + pitch) / 180) * cos(roll/ 180) + position_y;

	point[RD][X] = size/2 * pow(2, 0.5) * cos((45 - pitch) / 180) * cos(yaw / 180)  + position_x;
	point[RD][Y] =-size/2 * pow(2, 0.5) * sin((45 - pitch) / 180) * cos(roll/ 180) + position_y;
	
	ROS_INFO("roll = %lf , yaw = %lf , pitch = %lf, camera_pitch = %lf", roll, yaw, pitch,  camera_pitch); 

    ROS_INFO("point[LU][X] = %lf, point[LU][Y] = %lf ", point[LU][X], point[LU][Y]);
    ROS_INFO("point[RU][X] = %lf, point[RU][Y] = %lf ", point[RU][X], point[RU][Y]);
    ROS_INFO("point[LD][X] = %lf, point[LD][Y] = %lf ", point[LD][X], point[LD][Y]);
    ROS_INFO("point[RD][X] = %lf, point[RD][Y] = %lf ", point[RD][X], point[RD][Y]); 
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "aruco_listener");

	ros::NodeHandle n;
	ROS_INFO("START 1");
	ros::Subscriber sub = n.subscribe("/aruco_single/pose", 10, aruco_poseCallback);
	ROS_INFO("START 2");

	ros::spin();

	return 0;
}
