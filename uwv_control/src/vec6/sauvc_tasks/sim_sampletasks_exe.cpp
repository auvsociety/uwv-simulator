#include "sim_sampletasks_exe.h"

int main(int argc, char** argv)
{
  // initializing ROS node
  ros::init(argc, argv, "sampletasks");
  ros::NodeHandle nh;

  // initialize the underwater vehicle controller
  g_vec6.initController(nh, 40, 0.4);

  // Creating tasks
  TaskQueue tq;
  tq.addTask(std::make_shared<Task1>(g_vec6));
  tq.addTask(std::make_shared<Task2>(g_vec6));
  tq.addTask(std::make_shared<Task3>(g_vec6));

  // set the vehicle to traversing mode
  g_vec6.state_.is_traversing_ = true;

  // start the PID thread
  g_vec6.checkPidThread();
  
  g_vec6.allThrustersStop();

  // execute all the tasks in the pipeline
  while(!tq.pipeline_.empty()){
    tq.executeFirstTask();
  }

  // change the traversing state
  g_vec6.state_.is_traversing_ = false;
  g_vec6.checkPidThread();      // must call before the main function ends

  // stop all the thrusters
  g_vec6.allThrustersStop();

  ROS_INFO_STREAM("All sample tasks done!");
}

// MUST add
void signalHandler(int signum)
{
  g_vec6.state_.is_traversing_ = false;
  g_vec6.checkPidThread();
  exit(0);
}