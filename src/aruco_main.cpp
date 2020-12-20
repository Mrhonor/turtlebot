
#include "aruco_listener.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "aruco_listener_core");

	ros::NodeHandle n;
	ROS_INFO("START ");
	aruco_listener_core aruco_node(n);

	
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

	return 0;
}
