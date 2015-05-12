// Selective ros2lcm translator
// mfallon
// two modes:
// - passthrough: produces POSE_BODY and EST_ROBOT_STATE and CAMERA_LEFT
// - state estimation: produces ATLAS_STATE and POSE_BDI
//
// both modes produce SCAN

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <map>
#include <lcm/lcm-cpp.hpp>
#include <Eigen/Dense>

#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

#include <ihmc_msgs/BatchRawImuData.h>
#include <ihmc_msgs/RawImuData.h>
//#include <pronto_msgs/CachedRawIMUData.h>
//#include <pronto_msgs/RawIMUData.h>
//#include <pronto_msgs/FootSensor.h>

#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/pronto/force_torque_t.hpp"
#include "lcmtypes/pronto/robot_state_t.hpp"
#include "lcmtypes/pronto/atlas_state_t.hpp"
#include "lcmtypes/pronto/utime_t.hpp"
#include "lcmtypes/pronto/atlas_raw_imu_batch_t.hpp"
#include "lcmtypes/pronto/atlas_behavior_t.hpp"
#include "lcmtypes/pronto/multisense_state_t.hpp"
#include "lcmtypes/mav/ins_t.hpp"

#include <tf/transform_listener.h>

#define MODE_PASSTHROUGH 0
#define MODE_STATE_ESTIMATION 1

using namespace std;


class App{
public:
  App(ros::NodeHandle node_, int mode_, std::string robotName_, std::string imuSensor_);
  ~App();

private:
  lcm::LCM lcmPublish_ ;
  ros::NodeHandle node_;
  int mode_;
  string robotName_;
  string imuSensor_;

//  tf::TransformListener listener_;
  
  ros::Subscriber imuSensorSub_;

  void imuSensorCallback(const sensor_msgs::ImuConstPtr& msg);


  int64_t lastJointStateUtime_;
  bool verbose_;
};

App::App(ros::NodeHandle node_, int mode_, std::string robotName_, std::string imuSensor_) :
    node_(node_), mode_(mode_), robotName_(robotName_), imuSensor_(imuSensor_){
  ROS_INFO("Initializing Translator");
  if(!lcmPublish_.good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  // setting a queue of 1 reduces clumping of messages [desired for state estimation] but significantly reduces frequency
  // this is true even for just 2 subscriptions
  // setting to 100 means no messages are dropped during translation but imu data tends to be clumped into 50 message blocks

  int queue_size = 100;

  imuSensorSub_ = node_.subscribe(string( imuSensor_), queue_size, &App::imuSensorCallback,this);

  verbose_ = false;
//  listener_;
};

App::~App()  {
}




void App::imuSensorCallback(const sensor_msgs::ImuConstPtr& msg){

  mav::ins_t imu;
  imu.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  imu.device_time = imu.utime;
  imu.gyro[0] = msg->angular_velocity.x;
  imu.gyro[1] = msg->angular_velocity.y;
  imu.gyro[2] = msg->angular_velocity.z;
  imu.mag[0] = 0;
  imu.mag[1] = 0;
  imu.mag[2] = 0;
  imu.accel[0] = msg->linear_acceleration.x;
  imu.accel[1] = msg->linear_acceleration.y;
  imu.accel[2] = msg->linear_acceleration.z;
  imu.quat[0] = msg->orientation.w;
  imu.quat[1] = msg->orientation.x;
  imu.quat[2] = msg->orientation.y;
  imu.quat[2] = msg->orientation.z;
  imu.pressure = 0;
  imu.rel_alt = 0;

  lcmPublish_.publish( ("MICROSTRAIN_INS") , &imu);
}




int main(int argc, char **argv){
  std::string robotName;// = "valkyrie"; // "atlas"
  std::string modeArgument;
  std::string imuSensor = "Imu";


  if (argc >= 4){
     modeArgument = argv[1];
     robotName = argv[2];
     imuSensor = argv[3];
  }else {
    ROS_ERROR("Need to have three arguments: mode, robotName imuSensor");
    exit(-1);
  }

  int mode; // MODE_PASSTHROUGH or MODE_STATE_ESTIMATION
  if (modeArgument.compare("passthrough") == 0){
    mode = MODE_PASSTHROUGH;
  }else if (modeArgument.compare("state_estimation") == 0){
    mode = MODE_STATE_ESTIMATION;
  }else {
    ROS_ERROR("modeArgument not understood: use passthrough or state_estimation");
    exit(-1);
  }

  ros::init(argc, argv, "ros2lcm");
  ros::NodeHandle nh;
  new App(nh, mode, robotName, imuSensor);
  ROS_ERROR("ROS2LCM Translator Ready [mode: %d, %s] [robotName: %s] [imuSensor: %s]", mode, modeArgument.c_str(), robotName.c_str(), imuSensor.c_str());
  ros::spin();
  return 0;
}
