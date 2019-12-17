//
// Created by zft on 19-12-15.
//

#include "../include/multi_offboard/multi_test.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

//step 1: examine uav whether connect;

mavros_msgs::State uav1_current_state;
void uav1_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav1_current_state = *msg;
}
geometry_msgs::PoseStamped uav1_current_local_pos;
void uav1_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav1_current_local_pos = *msg;
}

mavros_msgs::State uav2_current_state;
void uav2_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav2_current_state = *msg;
}

geometry_msgs::PoseStamped uav2_current_local_pos;
void uav2_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav2_current_local_pos = *msg;
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "muti_board");
    ros::NodeHandle nh;
    /***setting the ***/
    ros::Subscriber uav1_state_sub = nh.subscribe<mavros_msgs::State>
            ("uav1/mavros/state", 10, uav1_state_cb);
    ros::Subscriber uav2_state_sub = nh.subscribe<mavros_msgs::State>
            ("uav2/mavros/state", 10, &uav2_state_cb);

    ros::Subscriber uav1_local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav1/mavros/local_position/pose", 10, uav1_local_pos_cb);
    ros::Subscriber uav2_local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav2/mavros/local_position/pose", 10, uav2_local_pos_cb);

    ros::Publisher uav1_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav1/mavros/setpoint_position/local", 10);
    ros::Publisher uav2_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav2/mavros/setpoint_position/local", 10);

    ros::ServiceClient uav1_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav1/mavros/cmd/arming");
    ros::ServiceClient uav2_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav2/mavros/cmd/arming");

    ros::ServiceClient uav1_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav1/mavros/set_mode");
    ros::ServiceClient uav2_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav2/mavros/set_mode");

    ros::Rate rate(20.0);
    /***test the usa1 whether is connected***/
    while (ros::ok() && !uav1_current_state.connected &&!uav2_current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose1;
    pose1.pose.position.x = 0;
    pose1.pose.position.y = 0;
    pose1.pose.position.z = 3;

    geometry_msgs::PoseStamped pose2;
    pose2.pose.position.x = 0;
    pose2.pose.position.y = 0;
    pose2.pose.position.z = 5;

    ROS_INFO("UAV is setting mode");
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {
        if (uav1_current_state.mode != "OFFBOARD" &&uav2_current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (uav1_set_mode_client.call(offb_set_mode) &&uav2_set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!uav1_current_state.armed &&!uav2_current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (uav1_arming_client.call(arm_cmd) &&uav2_arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        uav1_local_pos_pub.publish(pose1);
        uav2_local_pos_pub.publish(pose2);
        ros::spinOnce();
        rate.sleep();
}
}
