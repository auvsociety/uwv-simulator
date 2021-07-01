#include "vec6_hardwarecontroller.h"

void Vec6HardwareController::initController(ros::NodeHandle& _node, double _max_thrust, double _beta){
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

	ROS_INFO_STREAM("vec6's HardwareController is set.");
}

void Vec6HardwareController::heavePid2Effort(double _pid_heave){
	ROS_ERROR_STREAM("vec6's HardwareController NOT FULLY DEFINED.");
}

void Vec6HardwareController::vectoredPid2Effort(double _pid_surge, double _pid_yaw, double _pid_sway){
	ROS_ERROR_STREAM("vec6's HardwareController NOT FULLY DEFINED.");
}

void Vec6HardwareController::sendCommands(void){
  ROS_ERROR_STREAM("vec6's HardwareController NOT FULLY DEFINED.");
}

void Vec6HardwareController::allThrustersStop(void)
{
  ROS_ERROR_STREAM("vec6's HardwareController NOT FULLY DEFINED.");
}

void Vec6HardwareController::setEffort(double *_manual_effort)
{
  ROS_ERROR_STREAM("vec6's HardwareController NOT FULLY DEFINED.");
}