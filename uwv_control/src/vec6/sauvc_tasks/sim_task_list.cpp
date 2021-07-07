#include "sim_task_list.h"

// void scanObstacle()
// {
//   if (direction_flag == MOVE_LEFT)
//   {
//     if (my_auv_.set_orient_.yaw >= -45)
//     {
//       move::doYaw(my_auv_.set_orient_.yaw - 6, 1);
//     }
//     else if (my_auv_.set_orient_.yaw < -45)
//     {
//       direction_flag = MOVE_RIGHT;
//     }
//   }

//   if (direction_flag == MOVE_RIGHT)
//   {
//     if (my_auv_.set_orient_.yaw <= 45)
//     {
//       move::doYaw(my_auv_.set_orient_.yaw + 6, 1);
//     }
//     else
//     {
//       // Obstacle not found after performing yaw
//       move::doYaw(0, 1);

//       // Performing surge 
//       move::doSurge(28, 6);
//       direction_flag = MOVE_LEFT;
//     }
//   }
// }


void Task1::execute(){
	bool task_done = false;
	while(ros::ok() && !task_done){
		vec6_->doHeave(1.2, 0.01);

		// add a bit of delay for heave to stabilize
		vec6_->spinningDelay(8);

		vec6_->doYaw(45, 0.01);
		ros::spinOnce();

		task_done = true;
	}
	ROS_INFO_STREAM("Executed task 1!");
}

void Task2::execute(){
	bool task_done = false;
	while(ros::ok() && !task_done){
		vec6_->doSurge(20, 3);
		ros::spinOnce();

		task_done = true;
	}
	ROS_INFO_STREAM("Executed task 2!");
}

void Task3::execute(){
	bool task_done = false;
	while(ros::ok() && !task_done){
		vec6_->doSway(20, 3);
		ros::spinOnce();

		task_done = true;
	}
	ROS_INFO_STREAM("Executed task 3!");
}