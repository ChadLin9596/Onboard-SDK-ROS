#ifndef HELPING_FUNCTION_H
#define HELPING_FUNCTION_H

#include <ros/ros.h>
#include "dji_sdk/dji_sdk_node.h"
#include "dji_sdk/dji_sdk.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/tf.h"

char getch();

bool takeoff_land(int task);

bool obtain_control();

bool is_M100();

double confineHorizontal(double cmd);

double confineVertical(double cmd);


geometry_msgs::PoseStamped keyboard_control(geometry_msgs::PoseStamped goal);
#endif // HELPING_FUNCTION_H
