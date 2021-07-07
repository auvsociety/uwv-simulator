#include "vec6_controller.h"

void Vec6Controller::initPidControllers()
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

  if (ros::param::has(DEPTH_WINDUP_TOPIC))
  {
    ros::param::get(DEPTH_WINDUP_TOPIC, depth_controller_.int_windup_limit_);
  }
  else
  {
    ROS_WARN_STREAM("Did not find parameter: " << DEPTH_WINDUP_TOPIC << " Using default value: 5");
    depth_controller_.int_windup_limit_ = 5;
  }

  if (ros::param::has(DEPTH_OUT_LIMIT_TOPIC))
  {
    ros::param::get(DEPTH_OUT_LIMIT_TOPIC, depth_controller_.output_limit_);
  }
  else
  {
    ROS_WARN_STREAM("Did not find parameter: " << DEPTH_OUT_LIMIT_TOPIC << " Using default value: 32");
    depth_controller_.output_limit_ = 32;
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

  if (ros::param::has(YAW_WINDUP_TOPIC))
  {
    ros::param::get(YAW_WINDUP_TOPIC, yaw_controller_.int_windup_limit_);
  }
  else
  {
    ROS_WARN_STREAM("Did not find parameter: " << YAW_WINDUP_TOPIC << " Using default value: 5");
    yaw_controller_.int_windup_limit_ = 5;
  }

  if (ros::param::has(YAW_OUT_LIMIT_TOPIC))
  {
    ros::param::get(YAW_OUT_LIMIT_TOPIC, yaw_controller_.output_limit_);
  }
  else
  {
    ROS_WARN_STREAM("Did not find parameter: " << YAW_OUT_LIMIT_TOPIC << " Using default value: 32");
    yaw_controller_.output_limit_ = 32;
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
                  << "Yaw sensitivity: " << yaw_controller_.snstvty_ << std::endl
                  << "Yaw integral windup limit: " << yaw_controller_.int_windup_limit_ << std::endl
                  << "Yaw output limit: " << yaw_controller_.output_limit_ << std::endl);
  ROS_INFO_STREAM("\nDepth P gain: " << depth_controller_.kp_ << std::endl
                  << "Depth I gain: " << depth_controller_.ki_ << std::endl
                  << "Depth D gain: " << depth_controller_.kd_ << std::endl
                  << "Depth sensitivity: " << depth_controller_.snstvty_ << std::endl
                  << "Depth integral windup limit: " << depth_controller_.int_windup_limit_ << std::endl
                  << "Depth output limit: " << depth_controller_.output_limit_ << std::endl);
}

void Vec6Controller::checkPidThread(void){
  if(state_.is_traversing_){
    // start the PID thread if the vehicle is in traversing mode and the
    // thread has not yet been started
    if(!pid_thread_.joinable()){
      pid_thread_ = std::thread([&](){
          reuseThread();
          run();
        });
    }
  }
  else{
    // stop the reusable thread
    if(!stopRequested()){
      stop();
    }
    if(pid_thread_.joinable()){
      pid_thread_.join();
    }
  }
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

void Vec6Controller::spinningDelay(double _time){
  unsigned long ticks = GET_COUNTDOWN_TICKS(_time);
  ros::Rate rate(VEC6_SPIN_RATE);
  while(ticks != 1 && ros::ok()){
    ros::spinOnce();
    ticks--;
    rate.sleep();
  }
}

void Vec6Controller::doYaw(double _angle, double _sensitivity){
  if (!state_.is_traversing_)
  {
    ROS_INFO_STREAM("vec6 is not in the traversing mode.");
    return;
  }

  state_.set_orient_.yaw = _angle;

  ROS_INFO_STREAM("Performing yaw... Set-point: " << _angle);

  ros::Rate rate(VEC6_SPIN_RATE);

  while(fabs(state_.set_orient_.yaw - state_.cur_orient_.yaw) >= _sensitivity && ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO_STREAM("Performed yaw.");
}

void Vec6Controller::doHeave(double _depth, double _sensitivity){
  if (!state_.is_traversing_)
  {
    ROS_INFO_STREAM("vec6 is not in the traversing mode.");
    return;
  }

  state_.set_loc_.z = _depth;

  ROS_INFO_STREAM("Performing heave... Set-point: " << _depth);

  ros::Rate rate(VEC6_SPIN_RATE);

  while(fabs(state_.set_loc_.z - state_.cur_loc_.z) >= _sensitivity && ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO_STREAM("Performed heave.");
}

void Vec6Controller::doSurge(double _surge_thrust, double _surge_time){
  if (!state_.is_traversing_)
  {
    ROS_INFO_STREAM("vec6 is not in the traversing mode.");
    return;
  }

  if(_surge_thrust >= 0){
    allThrustersStop();

    state_.set_loc_.x = _surge_thrust;
    ROS_INFO_STREAM("Performing surge with thrust: " << _surge_thrust << 
                    " for time: " << _surge_time << " sec(s).");

    spinningDelay(_surge_time);

    state_.set_loc_.x = 0;

    ROS_INFO_STREAM("Performed surge.");
  }
  else{
    ROS_WARN_STREAM("Negative time for surge effort.");
  }
}

void Vec6Controller::doSway(double _sway_thrust, double _sway_time){
  if (!state_.is_traversing_)
  {
    ROS_INFO_STREAM("vec6 is not in the traversing mode.");
    return;
  }

  if(_sway_time >= 0){
    allThrustersStop();

    state_.set_loc_.y = _sway_thrust;
    ROS_INFO_STREAM("Performing sway with thrust: " << _sway_thrust << 
                    " for time: " << _sway_time << " sec(s).");

    spinningDelay(_sway_time);

    state_.set_loc_.y = 0;

    ROS_INFO_STREAM("Performed sway.");
  }
  else{
    ROS_WARN_STREAM("Negative time for sway effort.");
  }
}

void Vec6Controller::doSurgeAndSway(double _surge_thrust, double _surge_time,
                                    double _sway_thrust, double _sway_time){
  
  if (!state_.is_traversing_)
  {
    ROS_INFO_STREAM("vec6 is not in the traversing mode.");
    return;
  }
    
  if(_surge_time >= 0 && _sway_time >= 0){
    ROS_INFO_STREAM("Performing surge with thrust: " << _surge_thrust << 
                    " for time: " << _surge_time << " sec(s).\n" << 
                    "Performing sway with thrust: " << _sway_thrust << 
                    " for time: " << _sway_time << " sec(s).");
    
    state_.set_loc_.x = _surge_thrust;
    state_.set_loc_.y = _sway_thrust;

    if(_surge_time >= _sway_time){
      spinningDelay(_sway_time);
      state_.set_loc_.y = 0;
      spinningDelay(_surge_time - _sway_time);
      state_.set_loc_.x = 0;
    }
    else{
      spinningDelay(_surge_time);
      state_.set_loc_.x = 0;
      spinningDelay(_sway_time - _surge_time);
      state_.set_loc_.y = 0;
    }

    ROS_INFO_STREAM("Performed surge and sway.");
  }
  else{
    ROS_WARN_STREAM("Negative time in doSurgeSway!");
  }
}