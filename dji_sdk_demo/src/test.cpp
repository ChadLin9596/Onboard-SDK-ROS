#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  ros::Time begin = ros::Time::now();

  ROS_INFO("Hello world! %f",begin);
}
