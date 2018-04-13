/** @file offb_ctrl.cpp
 *  @version 1
 *  @date March, 2018
 *
 *  @brief
 *  use keyboard to control
 *
 *  @Author Chad Lin
 *	NCRL
 */

#ifndef PROJECT_OFFB_CTRL_H
#define PROJECT_OFFB_CTRL_H

#endif //PROJECT_OFFB_CTRL_H

#include <dji_sdk/SetLocalPosRef.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
//DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>

float target_offset_x;
float target_offset_y;
float target_offset_z;
float target_yaw;
double x ,y ,z, yaw;
double xCmd, yCmd, zCmd;
sensor_msgs::Joy controlPosYaw;
geometry_msgs::PoseStamped host_mocap;

bool obtain_control();

void host_pos(const geometry_msgs::PoseStamped::ConstPtr& msg);

char getch();

void keyboard_control();

void setTarget(float x, float y, float z, float yaw);