/**
 * @brief This header contains constants defining the various ROS parameters,
 * custom structures, and constants.
 */

#ifndef VEC6_CONFIG_H
#define VEC6_CONFIG_H

#include <mutex>

/// @brief If USE_DARKNET is defined, darknet_ros will be used
// #define USE_DARKNET

#define RAD2DEG 57.2957795131

/// @brief Total number of thrusters present in vec6
#define THRUSTER_NUM 6

/// @brief Thruster location and index mapping
#define F_PORT 0 
#define F_STAR 1
#define M_PORT 2
#define M_STAR 3
#define B_PORT 4
#define B_STAR 5

/// Depth PID parameter topic names
#define DEPTH_P_TOPIC "/uwv/vec6/depth_p"				
#define DEPTH_I_TOPIC "/uwv/vec6/depth_i"				
#define DEPTH_D_TOPIC "/uwv/vec6/depth_d"				
#define DEPTH_SNSTVTY_TOPIC "/uwv/vec6/depth_snstvty"

/// Yaw PID parameter topic names
#define YAW_P_TOPIC "/uwv/vec6/yaw_p"
#define YAW_I_TOPIC "/uwv/vec6/yaw_i"
#define YAW_D_TOPIC "/uwv/vec6/yaw_d"
#define YAW_SNSTVTY_TOPIC "/uwv/vec6/yaw_snstvty"

/// Message Topics
#define THRUSTER_EFFORT_TOPIC     "/uwv/vec6/thruster_command"
#define DEPTH_TOPIC               "/uwv/vec6/sim_depth"
#define IMU_TOPIC                 "/uwv/vec6/ekf_imu"
#define BOUNDING_BOX_TOPIC        "/darknet_ros/bounding_boxes"

/// @brief For thread safe sharing
std::mutex g_mtx;

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
 * Class with static members storing information about the current, and set
 * location and orientation of vec6 underwater vehicle
 */ 
class Vec6State
{
public:
  /// @brief Current location
  static Coordinates cur_loc_;

  /// @brief Set-point location
  static Coordinates set_loc_;

  /// @brief Current orientation
  static Orientation cur_orient_;
  
  /// @brief Set-point orientation
  static Orientation set_orient_;
};

#endif