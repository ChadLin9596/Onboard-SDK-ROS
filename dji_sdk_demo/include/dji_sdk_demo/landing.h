/** @file landing.cpp
 *  @version 1
 *  @date March, 2018
 *
 *  @brief
 *  automatic landing M100
 *
 *  @Author Chad Lin
 *	NCRL
 */

#ifndef PROJECT_LANDING_H
#define PROJECT_LANDING_H

#endif //PROJECT_LANDING_H

#include <ros/ros.h>
//DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>

bool takeoff_land(int task);

bool obtain_control();

bool is_M100();


