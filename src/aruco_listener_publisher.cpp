#include "aruco_listener_publisher.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "aruco_listener.h"
#include "aruco_listener/aruco_msg.h"

using std::string;

aruco_listener_publisher::aruco_listener_publisher(ros::NodeHandle &n, aruco_listener_core* Subject_):Subject(Subject_){
    if(Subject != nullptr){
        pub_RobotInfo = n.advertise<aruco_listener::aruco_msg>("/aruco_listener/robot/info", 10);

        // string MoveTopicName = string("/") + Subject->RobotName + string("/mobile_base/commands/velocity");
        string MoveTopicName = string("/mobile_base/commands/velocity");
        pub_turtlebot_move = n.advertise<geometry_msgs::Twist>(MoveTopicName, 10);

        string TrajectoryTopicName = string("/") + Subject->RobotName + string("/aruco_listener/trajectory");
        pub_trajectory = n.advertise<nav_msgs::Path>(TrajectoryTopicName, 10);
    }
    pub_Waiting = n.advertise<aruco_listener::aruco_msg>("/aruco_listener/robot/wait", 10);
}

void aruco_listener_publisher::PublishAll(){
    geometry_msgs::Twist msg;
    msg.linear.x = fabs(Subject->LinearV(0,0)) > 0.0001 ? Subject->LinearV(0,0) : 0;
    msg.linear.y = fabs(Subject->LinearV(1,0)) > 0.0001 ? Subject->LinearV(1,0) : 0;
    msg.linear.z = fabs(Subject->LinearV(2,0)) > 0.0001 ? Subject->LinearV(2,0) : 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    // msg.angular.z = 0;
    msg.angular.z = (fabs(Subject->AngularW(2,0)) > 0.0001) ? Subject->AngularW(2,0) : 0;
    
    pub_turtlebot_move.publish(msg);

    aruco_listener::aruco_msg msg_info;

    msg_info.robotName = Subject->RobotName;
    msg_info.x = Subject->CurrentCoordinate(0,0);
    msg_info.y = Subject->CurrentCoordinate(1,0);
    msg_info.yaw = Subject->Yaw;
    msg_info.angular.z = Subject->YawSpeed;
    msg_info.linear.x = Subject->CurrentLinearV(0,0);
    msg_info.linear.y = Subject->CurrentLinearV(1,0);
    msg_info.linear.z = Subject->CurrentLinearV(2,0);


    pub_RobotInfo.publish(msg_info);
}

void aruco_listener_publisher::PublishWaitingInfo(double x, double y, bool status){
    aruco_listener::aruco_msg msg;
    msg.robotName = Subject->RobotName;
    msg.x = x;
    msg.y = y;
    if(status == true){
        msg.yaw = 0;
    }
    else{
        msg.yaw = 1;
    }
    pub_Waiting.publish(msg);
}

void aruco_listener_publisher::PublishTrajectory(double x, double y){
    
    geometry_msgs::PoseStamped msg_pose;
    msg_pose.pose.position.x = x;
    msg_pose.pose.position.y = y;
    msg_pose.header.stamp = ros::Time::now();
    msg_pose.header.frame_id = "base";
    path_msg.header.frame_id="base";
    path_msg.poses.push_back(msg_pose);
    pub_trajectory.publish(path_msg);

}