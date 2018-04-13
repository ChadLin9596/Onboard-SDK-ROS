/** @file takeoff.cpp
 *  @version 1
 *  @date March, 2018
 *
 *  @brief
 *  automatic takeoff M100
 *
 *  @Author Chad Lin
 *	NCRL
 */

#include "dji_sdk_demo/takeoff.h"
#include "dji_sdk/dji_sdk.h"

ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "takeoff");
  ros::NodeHandle nh;

  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");

  bool obtain_control_result = obtain_control();

  if(obtain_control_result)
  {
  	ROS_INFO("Obtain Control Success!");
  }
  else
  {
  	ROS_INFO("Obtain Control Fail!");
  }
// take off
  
  if(is_M100())
  {
    ROS_INFO("M100 taking off!");
    takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF); // TASK_TAKEOFF = 4 
  }
  else
  {
  	ROS_INFO("Not M100!");
  }

  ros::spinOnce();
  return 0;
}