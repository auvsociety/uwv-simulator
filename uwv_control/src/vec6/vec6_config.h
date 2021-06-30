#ifndef VEC6_CONFIG_H
#define VEC6_CONFIG_H

/**
 * @brief This header contains constants defining the various ROS parameters.
 */


///> Depth PID parameter topic names
#define DEPTH_P_TOPIC "/uwv/vec6/depth_p"				
#define DEPTH_I_TOPIC "/uwv/vec6/depth_i"				
#define DEPTH_D_TOPIC "/uwv/vec6/depth_d"				
#define DEPTH_SNSTVTY_TOPIC "/vec6/depth_snstvty"

///> Yaw PID parameter topic names
#define YAW_P_TOPIC "/uwv/vec6/yaw_p"
#define YAW_I_TOPIC "/uwv/vec6/yaw_i"
#define YAW_D_TOPIC "/uwv/vec6/yaw_d"
#define YAW_SNSTVTY_TOPIC "/uwv/vec6/yaw_snstvty"

#endif