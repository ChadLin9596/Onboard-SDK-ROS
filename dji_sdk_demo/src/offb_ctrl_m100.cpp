#include "dji_sdk/dji_sdk.h"
#include "dji_sdk_demo/offb_ctrl.h"

void getTime(){
  double now =ros::Time::now().toSec();
  secs += now-start;
  start = now;
  if (secs >= 60)
  {
    secs -= 60;
    min += 1;
  }
  ROS_INFO("%2.2f min %2.2f sec",min,secs);
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
      goal.x += 0.05;
      break;
      case 115:     // key back   s
      goal.x -= 0.05;
      break;
      case 97:      // key left    a
      goal.y += 0.05;
      break;
      case 100:     // key right   d
      goal.y -= 0.05;
      break;
      case 114:     // key return  r
      goal.x = 0;
      goal.y = 0;
      goal.z = 0;
      break;
      case 105:    // i
      yawDesiredRad -= 0.01;
      break;
      case 117:    // u
      yawDesiredRad += 0.01;
      break;
    }
  }
}

void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  local_position = *msg;
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
  if (cmd >= 100)
  cmd = 100;
  else if (cmd <= 0)
  cmd = 0;
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "GPS_ctrl");
  ros::NodeHandle nh;

  ros::Subscriber host_mocap;
  ros::Publisher ctrlvelPub;
  ros::Publisher pidPub;

  host_mocap = nh.subscribe<geometry_msgs::PoseStamped>("vrpn_client_node/RigidBody1/pose", 10, &mocap_cb);
  ctrlvelPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  pidPub = nh.advertise<geometry_msgs::Quaternion>("PIDtuning",10);

  bool obtain_control_result = obtain_control();
  //initialize
  bias = 32;
  frequency = 100;
  double z_desire = 1; //m
  //vertical            horizontal_roll           horizontal_pitch
  pid_ver.KP = 20;      pid_hor_roll.KP = 0.5;    pid_hor_pitch.KP = 0.5;
  pid_ver.KI = 0.05;    pid_hor_roll.KI = 0.001;  pid_hor_pitch.KI = 0.001;
  pid_ver.KD = 1000;    pid_hor_roll.KD = 40;     pid_hor_pitch.KD = 40;
  pid_ver.in = 0;       pid_hor_roll.in = 0;      pid_hor_pitch.in = 0;
  pid_ver.de = 0;       pid_hor_roll.de = 0;      pid_hor_pitch.de = 0;
  pid_ver.pr = 0;       pid_hor_roll.pr = 0;      pid_hor_pitch.pr = 0;
  // yaw rate
  pid_yaw.KP = 0;
  pid_yaw.KI = 0;
  pid_yaw.KD = 0;
  pid_yaw.in = 0;
  pid_yaw.de = 0;
  pid_yaw.pr = 0;

  start =ros::Time::now().toSec();
 ros::Rate loop_rate(frequency);
 local_position.pose.orientation.w = 1;

while(ros::ok()){
  if(obtain_control_result)
  {
    ROS_INFO("---keyboard control start---");
    getTime();
    //ROS_INFO("i,u ctrl bias: %f",bias);
    //ROS_INFO("PID_thrust   : %f ,%f ,%f",pid_ver.KP,pid_ver.KI,pid_ver.KD);
    //ROS_INFO("PID_roll     : %f ,%f ,%f",pid_hor_roll.KP,pid_hor_roll.KI,pid_hor_roll.KD);
    //ROS_INFO("PID_pitch    : %f ,%f ,%f",pid_hor_pitch.KP,pid_hor_pitch.KI,pid_hor_pitch.KD);
    // Quaternion
    local_quat.x = local_position.pose.orientation.x;
    local_quat.y = local_position.pose.orientation.y;
    local_quat.z = local_position.pose.orientation.z;
    local_quat.w = local_position.pose.orientation.w;
    tf::Quaternion q(local_quat.x,local_quat.y,local_quat.z,local_quat.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    //std::cout << "Roll:" << roll << ",Pitch:" << pitch << ",Yaw:" << yaw << std::endl;
    ROS_INFO("Roll : %.4f Pitch : %.4f Yaw : %.4f",roll,pitch,yaw); // rad

    // translate to body frame
    /*
     * 2-D Rotation Matrix
     * cos(theta) -sin(theta)
     * sin(theta)  cos(theta)
     */
    local_body.x = cos(-yaw)*local_position.pose.position.x - sin(-yaw)*local_position.pose.position.y;
    local_body.y = sin(-yaw)*local_position.pose.position.x + cos(-yaw)*local_position.pose.position.y;
    local_body.z = local_position.pose.position.z;
    ROS_INFO("local position: %.4f, %.4f, %.4f",local_body.x,local_body.y,local_body.z);

    keyboard_control();
    err.pose.position.x = goal.x - local_body.x;
    err.pose.position.y = goal.y - local_body.y;
    err.pose.position.z = goal.z + z_desire - local_body.z;
    yaw_err = 0 - yaw;
    ROS_INFO("goal position : %.4f, %.4f, %.4f",goal.x,goal.y,goal.z);
    ROS_INFO("err  position : %.4f, %.4f, %.4f",err.pose.position.x,err.pose.position.y,err.pose.position.z);

    //vertical conmmands =================================
    pid_ver.in  = pid_ver.in + err.pose.position.z;
    //================
//    confine = 5;
//    if (pid_ver.in>confine)
//      pid_ver.in=confine;
//    else if (pid_ver.in<-confine)
//      pid_ver.in = -confine;
    //===============
    pid_ver.de = (err.pose.position.z - pid_ver.pr);
    pid_ver.pr =err.pose.position.z;

    // horizontal commands roll ============================;
    pid_hor_roll.in  = pid_hor_roll.in + err.pose.position.y;
    //===============
//    confine = 0.1;
//    if (pid_hor_roll.in>confine)
//      pid_hor_roll.in = confine;
//    else if (pid_hor_roll.in<-confine)
//      pid_hor_roll.in = -confine;
    //============
    pid_hor_roll.de = err.pose.position.y - pid_hor_roll.pr;
    pid_hor_roll.pr =err.pose.position.y;

    // horizontal commands pitch ============================
    pid_hor_pitch.in   = pid_hor_pitch.in + err.pose.position.x;
    //===============
//    confine = 0.1;
//    if (pid_hor_pitch.in>confine)
//      pid_hor_pitch.in=confine;
//    else if (pid_hor_pitch.in<-confine)
//      pid_hor_pitch.in = -confine;
    //===============
    pid_hor_pitch.de = err.pose.position.x - pid_hor_pitch.pr;
    pid_hor_pitch.pr = err.pose.position.x;

    // horizontal commands yawrate ============================
//    pid_yaw.in   = pid_yaw.in + err.pose.position.x;
//    //===============
////    confine = 0.1;
////    if (pid_hor_pitch.in>confine)
////      pid_hor_pitch.in=confine;
////    else if (pid_hor_pitch.in<-confine)
////      pid_hor_pitch.in = -confine;
//    //===============
//    pid_yaw.de = err.pose.position.x - pid_yaw.pr;
//    pid_yaw.pr = err.pose.position.x;


    rollCmd = -(err.pose.position.y*pid_hor_roll.KP + pid_hor_roll.in*pid_hor_roll.KI + pid_hor_roll.de*pid_hor_roll.KD);
    pitchCmd = err.pose.position.x*pid_hor_pitch.KP + pid_hor_pitch.in*pid_hor_pitch.KI + pid_hor_pitch.de*pid_hor_pitch.KD;
    thrustCmd = err.pose.position.z*pid_ver.KP + pid_ver.in*pid_ver.KI+ pid_ver.de*pid_ver.KD + bias;
    rollCmd = confineHorizontal(rollCmd);
    pitchCmd = confineHorizontal(pitchCmd);
    thrustCmd = confineVertical(thrustCmd);
    ROS_INFO("roll : %f pitch :%f thrust :%f yawrate:%f",rollCmd ,pitchCmd ,thrustCmd, yawDesiredRad);

    sensor_msgs::Joy controldata;
    controldata.axes.push_back(rollCmd);
    controldata.axes.push_back(pitchCmd);
    controldata.axes.push_back(thrustCmd);
    controldata.axes.push_back(yawDesiredRad);
    controldata.axes.push_back(flag);
    ctrlvelPub.publish(controldata);

    //PID tuning
    geometry_msgs::Quaternion RESULT;
    RESULT.x = err.pose.position.z*pid_ver.KP;
    RESULT.y = pid_ver.in*pid_ver.KI;
    RESULT.z = pid_ver.de*pid_ver.KD;
    RESULT.w = thrustCmd ;
    pidPub.publish(RESULT);
    //PID tuning

    ros::spinOnce();
    loop_rate.sleep();
  }
}
  ROS_INFO("Flight Control Ending");
}
