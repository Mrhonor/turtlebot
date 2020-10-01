#include "aruco_listener_publisher.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include "aruco_listener.h"


aruco_listener_publisher::aruco_listener_publisher(ros::NodeHandle &n_, aruco_listener* Subject_):n(n_), Subject(Subject_){
    pub_result = n.advertise<sensor_msgs::Image>("/lion_eyes/result", 10);
    pub_turtlebot_move = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);
}

void aruco_listener_publisher::PublishAll(){
    if(Subject->aruco_img_ptr != nullptr){
        pub_result.publish(Subject->aruco_img_ptr->toImageMsg());
    }
    geometry_msgs::Twist msg;
    msg.linear.x = Subject->LinearV(2,0);
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = -Subject->AngularW(2,0);
    
    pub_turtlebot_move.publish(msg);
}
