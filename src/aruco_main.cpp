
#include "aruco_listener.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "aruco_listener");

	ros::NodeHandle n;
	ROS_INFO("START ");
	aruco_listener aruco_node(n);

	ros::spin();

	return 0;
}
