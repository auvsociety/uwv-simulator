/**
 * @brief This header contains constants defining the various ROS parameters,
 * custom structures, and constants.
 */

#ifndef VEC6_CONFIG_H
#define VEC6_CONFIG_H

// Editable >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

/// @brief If USE_DARKNET is defined, darknet_ros will be used
// #define USE_DARKNET

/// @brief Rate at which state of the bot is checked (hz)
#define VEC6_SPIN_RATE 100

/// Depth PID parameter topic names
#define DEPTH_P_TOPIC "/vec6/depth_p"				
#define DEPTH_I_TOPIC "/vec6/depth_i"				
#define DEPTH_D_TOPIC "/vec6/depth_d"				
#define DEPTH_SNSTVTY_TOPIC "/vec6/depth_snstvty"
#define DEPTH_WINDUP_TOPIC "/vec6/depth_windup"
#define DEPTH_OUT_LIMIT_TOPIC "/vec6/depth_pid_out_limit"

/// Yaw PID parameter topic names
#define YAW_P_TOPIC "/vec6/yaw_p"
#define YAW_I_TOPIC "/vec6/yaw_i"
#define YAW_D_TOPIC "/vec6/yaw_d"
#define YAW_SNSTVTY_TOPIC "/vec6/yaw_snstvty"
#define YAW_WINDUP_TOPIC "/vec6/yaw_windup"
#define YAW_OUT_LIMIT_TOPIC "/vec6/yaw_pid_out_limit"

/// Message Topics
#define THRUSTER_EFFORT_TOPIC     "/vec6/thruster_command"
#define DEPTH_TOPIC               "/vec6/sim_depth"
#define IMU_TOPIC                 "/vec6/sim_imu"
#define BOUNDING_BOX_TOPIC        "/darknet_ros/bounding_boxes"

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// Edit only if you know what you are doing >>>>>>>>>>>>>>>>>>>>

/// @brief Macro to used in creating delay without blocking ROS communication
#define GET_COUNTDOWN_TICKS(x) x*VEC6_SPIN_RATE

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
 * Class with members storing information about various aspects of the 
 * vec6 vehicle
 */ 
class Vec6State
{
public:
  /// @brief Whether or not vec6 is in traversing mode
  bool is_traversing_;

  /// @brief Maximum effort for the thrusters
  double max_thrust_;

  /// @brief Constant for vectored-thruster fusion
  double beta_;

  /**
   * @brief SYS(surge, yaw, sway) matrix.
   * A matrix containing multiplication factors for each thruster in the 
   * vectored configuration. This along with the fusion constant are used
   * to determine the effort for each of the thrusters.
   * Row #0: Surge factors 
   * Row #1: Yaw factors
   * Row #2: Sway factors
   * Column sequence: F_PORT, F_STAR, B_PORT, B_STAR
  */
  const double sys_mat_[3][4] = {{1,1,1,1},{1,-1,1,-1},{1,-1,-1,1}};
  
  /// @brief Current location
  Coordinates cur_loc_;

  /// @brief Set-point location
  Coordinates set_loc_;

  /// @brief Current orientation
  Orientation cur_orient_;
  
  /// @brief Set-point orientation
  Orientation set_orient_;

  /**
   * @brief Constructor to create instance of vec6's state using following
   * critical parameters
   * 
   * @param _max_thrust maximum effort for the thrusters (default: 40)
   * @param _beta constant for vectored-thruster fusion  (default: 0.4)
   */
  Vec6State(double _max_thrust = 40, double _beta = 0.4){
    is_traversing_ = false;

    max_thrust_ = _max_thrust;
    beta_ = _beta;
  }

};

#endif