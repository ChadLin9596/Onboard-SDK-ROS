#include <ros/ros.h>
#include "dji_sdk/dji_sdk_node.h"

#include "sensor_msgs/Joy.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include <geometry_msgs/PointStamped.h>
#include "tf/tf.h"
#include <dji_sdk/DroneTaskControl.h>

geometry_msgs::PoseStamped local_position;
geometry_msgs::PoseStamped err;
geometry_msgs::Quaternion local_quat;
geometry_msgs::Point goal;
geometry_msgs::Point local_body;

ros::ServiceClient sdk_ctrl_authority_service;

uint8_t flag = (
                DJISDK::VERTICAL_THRUST      | // VERTICAL_THRUST = 0~100%
                DJISDK::HORIZONTAL_ANGLE     | // limit 35 degree
                DJISDK::YAW_ANGLE            | // limit 150 degree/s
                DJISDK::HORIZONTAL_BODY      | // body frame
                DJISDK::STABLE_ENABLE
               );

struct PID
{
  float KP;
  float KI;
  float KD;
  float in;
  float de;
  float pr;
};

struct PID pid_ver;
struct PID pid_hor_roll;
struct PID pid_hor_pitch;
