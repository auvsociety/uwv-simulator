#include "vec6_controller.h"

void Vec6Controller::initControllers()
{
  // initialize controllers
  ROS_INFO_STREAM("Configuring PID controllers...");
  depth_controller_ = PidTranslate();
  yaw_controller_ = PidRotate();
  updatePidsGains();
  ROS_INFO_STREAM("PID gains updated.");

  ROS_INFO_STREAM("vec6 controllers are ready.");
}

void Vec6Controller::updatePidsGains(void)
{
  ROS_INFO_STREAM("Updating PID and sensitivity values...");

  // Depth
  if (ros::param::has(DEPTH_P_TOPIC))
  {
    ros::param::get(DEPTH_P_TOPIC, depth_controller_.kp_);
  }
  else
  {
    ROS_WARN_STREAM("Did not find parameter: " << DEPTH_P_TOPIC << " Using default value: 10");
    depth_controller_.kp_ = 10;
  }

  if (ros::param::has(DEPTH_I_TOPIC))
  {
    ros::param::get(DEPTH_I_TOPIC, depth_controller_.ki_);
  }
  else
  {
    ROS_WARN_STREAM("Did not find parameter: " << DEPTH_I_TOPIC << " Using default value: 0.1");
    depth_controller_.ki_ = 0.1;
  }

  if (ros::param::has(DEPTH_D_TOPIC))
  {
    ros::param::get(DEPTH_D_TOPIC, depth_controller_.kd_);
  }
  else
  {
    ROS_WARN_STREAM("Did not find parameter: " << DEPTH_D_TOPIC << " Using default value: 0.5");
    depth_controller_.kd_ = 0.5;
  }

  if (ros::param::has(DEPTH_SNSTVTY_TOPIC))
  {
    ros::param::get(DEPTH_SNSTVTY_TOPIC, depth_controller_.snstvty_);
  }
  else
  {
    ROS_WARN_STREAM("Did not find parameter: " << DEPTH_SNSTVTY_TOPIC << " Using default value: 0.01");
    depth_controller_.snstvty_ = 0.01;
  }

  // Yaw
  if (ros::param::has(YAW_P_TOPIC))
  {
    ros::param::get(YAW_P_TOPIC, yaw_controller_.kp_);
  }
  else
  {
    ROS_WARN_STREAM("Did not find parameter: " << YAW_P_TOPIC << " Using default value: 10");
    yaw_controller_.kp_ = 10;
  }

  if (ros::param::has(YAW_I_TOPIC))
  {
    ros::param::get(YAW_I_TOPIC, yaw_controller_.ki_);
  }
  else
  {
    ROS_WARN_STREAM("Did not find parameter: " << YAW_I_TOPIC << " Using default value: 0.1");
    yaw_controller_.ki_ = 0.1;
  }

  if (ros::param::has(YAW_D_TOPIC))
  {
    ros::param::get(YAW_D_TOPIC, yaw_controller_.kd_);
  }
  else
  {
    ROS_WARN_STREAM("Did not find parameter: " << YAW_D_TOPIC << " Using default value: 0.5");
    yaw_controller_.kd_ = 0.5;
  }

  if (ros::param::has(YAW_SNSTVTY_TOPIC))
  {
    ros::param::get(YAW_SNSTVTY_TOPIC, yaw_controller_.snstvty_);
  }
  else
  {
    ROS_WARN_STREAM("Did not find parameter: " << YAW_SNSTVTY_TOPIC << " Using default value: 0.01");
    yaw_controller_.snstvty_ = 0.01;
  }

  ROS_INFO_STREAM("Updated PID and sensitivity values.");
}

void Vec6Controller::resetControllers(bool _yaw, bool _depth){
  if(_yaw)   yaw_controller_.reset();
  if(_depth) depth_controller_.reset();
  updatePidsGains();
}

void Vec6Controller::showPidsGains(void){
  ROS_INFO_STREAM("\nYaw P gain: " << yaw_controller_.kp_ << std::endl
                  << "Yaw I gain: " << yaw_controller_.ki_ << std::endl
                  << "Yaw D gain: " << yaw_controller_.kd_ << std::endl
                  << "Yaw sensitivity: " << yaw_controller_.snstvty_ << std::endl);
  ROS_INFO_STREAM("\nDepth P gain: " << depth_controller_.kp_ << std::endl
                  << "Depth I gain: " << depth_controller_.ki_ << std::endl
                  << "Depth D gain: " << depth_controller_.kd_ << std::endl
                  << "Depth sensitivity: " << depth_controller_.snstvty_ << std::endl);
}

void Vec6Controller::run(){
  // variables for loop operation
  double prev_time = ros::Time::now().toSec();
  double cur_time;
  double dt;

  ROS_INFO_STREAM("Starting PID controller thread in background...");

  // Check if thread is requested to stop
  while (stopRequested() == false)
  {
    cur_time = ros::Time::now().toSec();

    dt = cur_time - prev_time;

    // sanity check
    if (dt <= 0)
      continue;
    heavePid2Effort(depth_controller_.update(state_.set_loc_.z, state_.cur_loc_.z, dt));
    vectoredPid2Effort(state_.set_loc_.x,
                       yaw_controller_.update(state_.set_orient_.yaw, state_.cur_orient_.yaw, dt),
                       state_.set_loc_.y);
    sendCommands();

    prev_time = cur_time;
  }
  ROS_INFO_STREAM("PID controller thread stopped.");
}