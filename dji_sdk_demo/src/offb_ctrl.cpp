#include <ros/ros.h>
#include "dji_sdk/dji_sdk_node.h"
#include "dji_sdk/dji_sdk.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include <geometry_msgs/PointStamped.h>
#include "tf/tf.h"
#define KP_horizontal 0.05
//#define PI 3.1415926  C_PI

geometry_msgs::Point goal;
geometry_msgs::PoseStamped host_mocap;
geometry_msgs::PoseStamped err;
geometry_msgs::Quaternion local_quat;
geometry_msgs::Point local_body;
// in GPS environment
geometry_msgs::PointStamped local_position;
ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
// in GPS environment
uint8_t flag = (
                DJISDK::VERTICAL_THRUST      | // VERTICAL_THRUST = 0~100%
                DJISDK::HORIZONTAL_ANGLE     | // limit 35 degree
                DJISDK::YAW_ANGLE            | // limit 150 degree/s
                DJISDK::HORIZONTAL_BODY      | // body frame
                DJISDK::STABLE_ENABLE
               );

struct PID
{
  double KP;
  double KI;
  double KD;
};
struct PID pid;
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
  //ROS_INFO("C: %d",c);
  if (c != EOF) {
    switch (c)
    {
      case 111:     // key up  o
      goal.z += 0.01;
      //ROS_INFO("z+!!!");
      break;
      case 112:     // key down p
      goal.z -= 0.01;
      //ROS_INFO("z-!!!");
      break;
      case 119:     // key foward  w
      goal.x += 0.001;
      break;
      case 115:     // key back   s
      goal.x -= 0.001;
      break;
      case 97:      // key left     a
      goal.y += 0.001;
      break;
      case 100:     // key right   d
      goal.y -= 0.001;
      break;
      case 114:     // key return  r
      goal.x = 0;
      goal.y = 0;
      goal.z = 0;
      break;
      case 106:     // j
      pid.KP += 0.001;
      break;
      case 107:     // k
      pid.KI += 0.001;
      break;
      case 108:     // l
      pid.KD += 0.001;
      break;
      case 109:     // m
      pid.KP -= 0.001;
      break;
      case 44 :     // ,
      pid.KI -= 0.001;
      break;
      case 46 :     // .
      pid.KD -= 0.001;
      break;
    }
  }
}

void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){

  host_mocap = *msg;
}

double confineHorizontal(double cmd)
{
  if (cmd >= 0.1)
  {
    ROS_INFO("cmd is too large");
        cmd = 0.1;
  }
  return cmd;
}

double confineVertical(double cmd)
{
  if (cmd >= 37.5)
  cmd = 37.3;
  else if (cmd <= 37.1)
  cmd = 37.1;
  else
  return cmd;
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
// in GPS environment
void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
  local_position = *msg;
}

bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
}
// in GPS environment
int main(int argc, char **argv)
{
  ros::init(argc, argv, "offb_ctrl");
  ros::NodeHandle nh;
  
  ros::Subscriber host_mocap;
  ros::Publisher ctrlvelPub;

  host_mocap = nh.subscribe<geometry_msgs::PoseStamped>("vrpn_client_node/RigidBody1/pose", 10, &mocap_cb);
  ctrlvelPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  // in GPS environment
  ros::Subscriber localPosition;
  localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");
  // in GPS environment

  //initialize
  int frequency = 50;
  float bias = 37;
  bool obtain_control_result = obtain_control();
  bool integtal = 0;
  bool derivative = 0;
  bool err_prior = 0;
  pid.KP = 0.5;
  pid.KI = 0;
  pid.KD = 0;
  double  rollCmd, pitchCmd, thrustCmd;
  double  yawDesiredRad= 0  ;
  double z_desire = 2; //m
  set_local_position();

  ros::Rate loop_rate(frequency);
  while(obtain_control_result){
    ROS_INFO("---keyboard control start---");
    //input goal;

    // Quaternion
//    local_quat.x = local_position.pose.orientation.x;
//    local_quat.y = local_position.pose.orientation.y;
//    local_quat.z = local_position.pose.orientation.z;
//    local_quat.w = local_position.pose.orientation.w;
//    tf::Quaternion q(local_quat.x,local_quat.y,local_quat.z,local_quat.w);
//    tf::Matrix3x3 m(q);
//    double roll, pitch, yaw;
//    m.getRPY(roll, pitch, yaw);
//    //std::cout << "Roll:" << roll << ",Pitch:" << pitch << ",Yaw:" << yaw << std::endl;
//    ROS_INFO("Roll : %.4f Pitch : %.4f Yaw : %.4f",roll,pitch,yaw); // rad
//    // translate to body frame
//    /*
//     * 2-D Rotation Matrix
//     * cos(theta) -sin(theta)
//     * sin(theta)  cos(theta)
//     */
//    local_body.x = cos(-yaw)*local_position.pose.position.x - sin(-yaw)*local_position.pose.position.y;
//    local_body.y = sin(-yaw)*local_position.pose.position.x + cos(-yaw)*local_position.pose.position.y;
//    local_body.z = local_position.pose.position.z;
    local_body.x = local_position.point.x;
    local_body.y = local_position.point.y;
    local_body.z = local_position.point.z;
    ROS_INFO("local position: %.4f, %.4f, %.4f",local_body.x,local_body.y,local_body.z);

    keyboard_control();
    err.pose.position.x = goal.x - local_body.x;
    err.pose.position.y = goal.y - local_body.y;
    err.pose.position.z = goal.z + z_desire - local_body.z;
    ROS_INFO("goal position : %.4f, %.4f, %.4f",goal.x,goal.y,goal.z);
    ROS_INFO("err  position : %.4f, %.4f, %.4f",err.pose.position.x,err.pose.position.y,err.pose.position.z);
    //conmmands
    ROS_INFO("P : %f I : %f  D : %f ",pid.KP,pid.KI,pid.KD);
    integtal   = integtal + err.pose.position.z*(1/frequency);
    derivative = (err.pose.position.z - err_prior)*frequency;
    err_prior =err.pose.position.z;

    rollCmd = -err.pose.position.y*KP_horizontal;
    pitchCmd = err.pose.position.x*KP_horizontal;
    thrustCmd = err.pose.position.z*pid.KP + integtal*pid.KI + derivative*pid.KD + bias;
    //rollCmd = confineHorizontal(rollCmd);
    //pitchCmd = confineHorizontal(pitchCmd);
    //thrustCmd = confineVertical(thrustCmd);
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
  ros::spinOnce();
}
