#ifndef VEC6_CONTROLLER_H
#define VEC6_CONTROLLER_H

#include <ros/ros.h>

// Control libraries
#include "pid_translate.h"
#include "pid_rotate.h"

// Misc libraries
#include "vec6_config.h"
#include "stoppable_thread.h"
#include <thread>

/**
 * @brief Abstract class that controls the PID loop and configuration
 */
class Vec6Controller : public StoppableThread
{
public:
  /// @brief Instance holding the vec6 vehicle's state
  Vec6State state_;

  /// @brief Depth controller
  PidTranslate depth_controller_;

  /// @brief Yaw controller
  PidRotate yaw_controller_;

  /// @brief PID thread to be controlled in the background
  std::thread pid_thread_;

  Vec6Controller(){}
  virtual ~Vec6Controller(){}

  /**
   * @brief Initializes the PID controllers, and updates the gain values. Not to be
   * called by an instance.
   */
  void initPidControllers(void);

  /**
   * @brief Updates the PID gain values and sensitivity value from the 
   * ROS parameter server. Default values are used if parameters are 
   * not set.
   * 
   * Default: kp: 10 | ki: 0.1 | kd: 0.5 | snstvty: 0.01 | integral windup limit: 5 |
   * maximum pid output: 32
   */
  void updatePidsGains(void);

  /**
   * @brief Resets PID controllers and updates the gain values from ROS
   * parameter server.
   * 
   * @param _yaw resets yaw PID controller if true
   * @param _depth resets yaw PID controller if true
   */
  void resetControllers(bool _yaw, bool _depth);

  /**
   * @brief Displays of PID gain and sensitivity values of all the controllers
   */ 
  void showPidsGains(void);

  /**
   * @brief Checks the PID thread and stops the thread if the vehicle is not
   * traversing. Must be called everytime in the main-loop and at the end of the
   * ROS node (before the main-thread stops)
   */ 
  void checkPidThread(void);

  /**
   * @brief Runs the PID loop in the background until 
   * Vec6Controller_obj.stop() called.
   */ 
  void run();

  /**
   * @brief Blocks the thread while executing ros::spinOnce()
   * @param _time Time for which this delay should occur (in seconds)
   */
  void spinningDelay(double _time);

  /**
   * @brief Performs yaw until the current position lies within the given sensitivity
   * 
   * @param _angle Set-point to be achieved
   * @param _sensitivity SET_POINT +- sensitivity -> defines the range in which if the bot it, it is considered as successful
   */ 
  void doYaw(double _angle, double _sensitivity);

  /**
   * @brief Performs heave until the current position lies within the given sensitivity
   * 
   * @param _depth Set-point to be achieved
   * @param _sensitivity SET_POINT +- sensitivity -> defines the range in which if the bot it, it is considered as successful
   */
  void doHeave(double _depth, double _sensitivity);

  /**
   * @brief Sets the given surge thrust for the given amount of time.
   * 
   * @param _surge_thrust Amount of surge thrust
   * @param _surge_time Time (in seconds) for which the thrust should be active
   */
  void doSurge(double _surge_thrust, double _surge_time);

  /**
   * @brief Sets the given sway thrust for the given amount of time.
   * 
   * @param _sway_thrust Amount of sway thrust
   * @param _sway_time Time (in seconds) for which the thrust should be active
   */
  void doSway(double _sway_thrust, double _sway_time);

  /**
   * @brief Sets the given surge and sway thrust simultaneously for 
   * the time allocated to both of them
   * 
   * @param _surge_thrust Amount of surge thrust
   * @param _surge_time Time (in seconds) for which the thrust should be active
   * @param _sway_thrust Amount of sway thrust
   * @param _sway_time Time (in seconds) for which the thrust should be active
   */
  void doSurgeAndSway(double _surge_thrust, double _surge_time, double _sway_thrust, double _sway_time);

  virtual void heavePid2Effort(double _pid_heave) = 0;
  virtual void vectoredPid2Effort(double _pid_yaw, double _pid_surge, double _pid_sway) = 0;
  virtual void sendCommands(void) = 0;
  virtual void allThrustersStop(void) = 0;
  virtual void setEffort(double *_manual_effort) = 0;
};

#endif
