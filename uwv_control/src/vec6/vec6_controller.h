#ifndef VEC6_CONTROLLER_H
#define VEC6_CONTROLLER_H

#include <ros/ros.h>

// Control libraries
#include "pid_translate.h"
#include "pid_rotate.h"

// Misc libraries
#include "vec6_config.h"
#include "stoppable_thread.h"
#include <cmath>
#define RAD2DEG 57.2957795131

/// @brief Total number of thrusters present in vec6
#define THRUSTER_NUM 6

/// @brief Structure to store roll, pitch, yaw (the orientation) of vec6
struct Orientation
{
  double roll, pitch, yaw;
};

/// @brief Structure to store x, y, z coordinates of vec6
struct Coordinates
{
  double x, y, z;
};

/**
 * @brief Abstract class that controls the PID loop and configuration
 */
class Vec6Controller : public StoppableThread
{
public:
  /// @brief Current location
  Coordinates cur_loc_;

  /// @brief Set-point location
  Coordinates set_loc_;

  /// @brief Current orientation
  Orientation cur_orient_;
  
  /// @brief Set-point orientation
  Orientation set_orient_;

  /// @brief Depth controller
  PidTranslate depth_controller_;

  /// @brief Yaw controller
  PidRotate yaw_controller_;

  virtual ~Vec6Controller(){}

  /**
   * @brief Initializes the PID controllers, and updates the gain values.
   * 
   * @param node Reference to the node handle of the UWV's controller
   */
  void initControllers(void);

  /**
   * @brief Updates the PID gain values and sensitivity value from the 
   * ROS parameter server. Default values are used if parameters are 
   * not set.
   * 
   * Default: p: 10 | i: 0.1 | d: 0.5 | snstvty: 0.01
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
   * @brief Runs the PID loop in the background until 
   * Vec6Controller_obj.stop() called.
   */ 
  void run();

  virtual void heavePid2Effort(float pid_heave) = 0;
  virtual void vectoredPid2Effort(float pid_yaw, float pid_surge, float pid_sway) = 0;
  virtual void sendCommands(void) = 0;
};

#endif
