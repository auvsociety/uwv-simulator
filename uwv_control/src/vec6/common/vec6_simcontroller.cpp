#include "vec6_simcontroller.h"

void Vec6SimController::initController(ros::NodeHandle& _node, double _max_thrust, double _beta){
	initPidControllers();

	// Setting default values
	state_.max_thrust_ = _max_thrust;
	state_.beta_ = _beta;

	effort_.name.resize(THRUSTER_NUM);
	effort_.effort.resize(THRUSTER_NUM);
	for (int i = 0; i < THRUSTER_NUM; i++)
	{
		effort_.effort[i] = 0.0;
	}
	effort_.name[F_PORT] = "f_port";
	effort_.name[F_STAR] = "f_star";
	effort_.name[M_PORT] = "m_port";
	effort_.name[M_STAR] = "m_star";
	effort_.name[B_PORT] = "b_port";
	effort_.name[B_STAR] = "b_star";

	state_.set_loc_.x = 0;
	state_.set_loc_.y = 0;
	state_.set_loc_.z = 0;
	state_.cur_loc_.x = 0;
	state_.cur_loc_.y = 0;
	state_.cur_loc_.z = 0;
	state_.set_orient_.yaw = 0;
	state_.set_orient_.pitch = 0;
	state_.set_orient_.roll = 0;
	state_.cur_orient_.yaw = 0;
	state_.cur_orient_.pitch = 0;
	state_.cur_orient_.roll = 0;

	state_.is_traversing_ = false;

	vec6Comms.initComms(_node, state_);

	ROS_INFO_STREAM("vec6's SimController is set.");
}

void Vec6SimController::heavePid2Effort(double _pid_heave){
	if (!state_.is_traversing_){
		ROS_INFO_STREAM("vec6 is not in the traversing mode.");
		return;
	}
    effort_.effort[M_PORT] = _pid_heave;
    effort_.effort[M_STAR] = _pid_heave;
}

void Vec6SimController::vectoredPid2Effort(double _pid_surge, double _pid_yaw, double _pid_sway){
	if (!state_.is_traversing_){
		ROS_INFO_STREAM("vec6 is not in the traversing mode.");
		return;
  	}

effort_.effort[F_PORT] = ((1-state_.beta_)/2)*state_.sys_mat_[0][0]*_pid_surge + state_.beta_*state_.sys_mat_[1][0]*_pid_yaw + ((1-state_.beta_)/2)*state_.sys_mat_[2][0]*_pid_sway;
effort_.effort[F_STAR] = ((1-state_.beta_)/2)*state_.sys_mat_[0][1]*_pid_surge + state_.beta_*state_.sys_mat_[1][1]*_pid_yaw + ((1-state_.beta_)/2)*state_.sys_mat_[2][1]*_pid_sway;
effort_.effort[B_PORT] = ((1-state_.beta_)/2)*state_.sys_mat_[0][2]*_pid_surge + state_.beta_*state_.sys_mat_[1][2]*_pid_yaw + ((1-state_.beta_)/2)*state_.sys_mat_[2][2]*_pid_sway;
effort_.effort[B_STAR] = ((1-state_.beta_)/2)*state_.sys_mat_[0][3]*_pid_surge + state_.beta_*state_.sys_mat_[1][3]*_pid_yaw + ((1-state_.beta_)/2)*state_.sys_mat_[2][3]*_pid_sway;
    
// limiting thrust
if(effort_.effort[F_PORT] > state_.max_thrust_) effort_.effort[F_PORT] = state_.max_thrust_ - 8; 
if(effort_.effort[F_STAR] > state_.max_thrust_) effort_.effort[F_STAR] = state_.max_thrust_ - 8;
if(effort_.effort[B_PORT] > state_.max_thrust_) effort_.effort[B_PORT] = state_.max_thrust_ - 8;
if(effort_.effort[B_STAR] > state_.max_thrust_) effort_.effort[B_STAR] = state_.max_thrust_ - 8;

}

void Vec6SimController::sendCommands(void){
  vec6Comms.effort_pub_.publish(effort_);
}

void Vec6SimController::allThrustersStop(void)
{
  for (int i = 0; i < THRUSTER_NUM; i++)
  {
    effort_.effort[i] = 0;
  }
  vec6Comms.effort_pub_.publish(effort_);
}

void Vec6SimController::setEffort(double *_manual_effort)
{
  if (!state_.is_traversing_)
  {
    ROS_INFO_STREAM("vec6 is not in the traversing mode.");
    return;
  }

  for (int i = 0; i < THRUSTER_NUM; i++)
  {
    effort_.effort[i] = *(_manual_effort + i);
  }
  vec6Comms.effort_pub_.publish(effort_);
}