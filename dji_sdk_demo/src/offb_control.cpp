#include <ros/ros.h>
#include "dji_sdk/dji_sdk_node.h"
#include "dji_sdk/dji_sdk.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/tf.h"
#define KP 0.001
#define KI 0.001
#define KD  0.001
#define KP_YAW 0.001
#define KP_z 1
#define KP_horizontal 0.01
#define PI 3.1415926
double z_desire = 0.5; //m

geometry_msgs::PoseStamped goal;
geometry_msgs::PoseStamped local_position;
geometry_msgs::PoseStamped err;
geometry_msgs::Quaternion local_quat;
geometry_msgs::Point local_body;
sensor_msgs::Joy controldata;
// ServiceClient
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;
// Publisher
ros::Publisher ctrlvelPub;

uint8_t flag = (
                DJISDK::VERTICAL_POSITION      |
                DJISDK::HORIZONTAL_ANGLE     |
                DJISDK::YAW_ANGLE            |
                DJISDK::HORIZONTAL_BODY      |
                DJISDK::STABLE_ENABLE
       );

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

char getch()
{
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) {
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0) {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) {
        perror ("tcsetattr ~ICANON");
    }
    return (buf);
}

void keyboard_control()
{

  int c = getch();
  ROS_INFO("C: %d",c);
  if (c != EOF) {
    switch (c)
    {
      case 111:    // key up  o
        goal.pose.position.z += 0.1;
        ROS_INFO("z+!!!");
        break;
      case 112:    // key down p
        goal.pose.position.z -= 0.1;
        ROS_INFO("z-!!!");
        break;
      case 119:    // key foward  w
        goal.pose.position.x += 0.001;
        ROS_INFO("x+!!!");
        break;
      case 115:    // key back   s
        goal.pose.position.x -= 0.001;
        ROS_INFO("x-!!!");
        break;
      case 97:    // key left     a
        goal.pose.position.y -= 0.001;
        ROS_INFO("y+!!!");
        break;
      case 100:    // key right   d
        goal.pose.position.y += 0.001;
        ROS_INFO("y-!!!");
        break;
      case 114:    // key return  r
        goal.pose.position.x = 0;
        goal.pose.position.y = 0;
        goal.pose.position.z = 0;
        ROS_INFO("Return home");
        break;

    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "offb_control");
  ros::NodeHandle nh;
  int test = 0;
  // Publisher
  ctrlvelPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
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
    test = 1;
  }
  else
  {
  	ROS_INFO("Not M100!");
  } 
  ros::Rate loop_rate(30);
// keyboard Control
  while(test){
    //input goal;

    // Quaternion
    local_quat.x = local_position.pose.orientation.x;
    local_quat.y = local_position.pose.orientation.y;
    local_quat.z = local_position.pose.orientation.z;
    local_quat.w = local_position.pose.orientation.w;
    tf::Quaternion q(local_quat.x,local_quat.y,local_quat.z,local_quat.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    std::cout << "Roll:" << roll << ",Pitch:" << pitch << ",Yaw:" << yaw << std::endl;
    printf("Yaw : %.4f",yaw); // rad
    //yaw = yaw+PI;
    // translate to body frame
    local_body.x = cos(-yaw)*local_position.pose.position.x - sin(-yaw)*local_position.pose.position.y;
    local_body.y = sin(-yaw)*local_position.pose.position.x + cos(-yaw)*local_position.pose.position.y;
    local_body.z = local_position.pose.position.z;
    ROS_INFO("local_body : %.4f, %.4f, %.4f",local_body.x,local_body.y,local_body.z);
    keyboard_control();
    err.pose.position.x = goal.pose.position.x ;//- local_body.x;
    err.pose.position.y = goal.pose.position.y ;//- local_body.y;
    err.pose.position.z = goal.pose.position.z + z_desire ;//- local_body.z;

    ROS_INFO("err_position : %.4f, %.4f, %.4f",err.pose.position.x,err.pose.position.y,err.pose.position.z);

    //conmmand
    double  rollCmd, pitchCmd, thrustCmd;
    double  yawDesiredRad= 0  ;

    rollCmd = -err.pose.position.y*KP_horizontal;
    pitchCmd = -err.pose.position.x*KP_horizontal;
    thrustCmd = err.pose.position.z*KP_z;

    ROS_INFO("roll : %.4f pitch :%.4f thrust :%.4f",rollCmd ,pitchCmd ,thrustCmd);

    sensor_msgs::Joy controldata;
    controldata.axes.push_back(rollCmd);
    controldata.axes.push_back(pitchCmd);
    controldata.axes.push_back(thrustCmd);
    controldata.axes.push_back(yawDesiredRad);
    controldata.axes.push_back(flag);
    ctrlvelPub.publish(controldata);

    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("Flight Control Ending");
}
