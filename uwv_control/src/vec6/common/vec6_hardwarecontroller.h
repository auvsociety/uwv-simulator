#ifndef VEC6_HARDWARECONTROLLER_H
#define VEC6_HARDWARECONTROLLER_H

#include <ros/ros.h>

// Message data-types
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include "vec6_config.h"
#include "vec6_controller.h"
#include "vec6_comms.h"


class Vec6HardwareController : public Vec6Controller{

public:
	/// @brief Instance taking care of the communication between the vehicle's
	/// sub-systems
	Vec6Comms vec6Comms;

	/// @brief Effort message
	sensor_msgs::JointState effort_;

	/**
	 * @brief Initializes the controller and the variables.
	 * 
	 * @param _node Reference to the node handle of vec6's main ROS node
	 * @param _max_thrust maximum effort for the thrusters
	 * @param _beta constant for vectored-thruster fusion
	 */
	void initController(ros::NodeHandle& _node, double _max_thrust, double _beta);
	
	virtual ~Vec6HardwareController(){}

	/**
	 * @brief Converts the output of PID into effort for motion along Z direction
	 * (heave)
	 * @param _pid_heave Output of PID
	 */
	void heavePid2Effort(double _pid_heave) override;

	/**
	 * @brief Converts the output of PIDs into effort for rotation along Z axis
	 * (yaw), surge, and sway as per the following equation:
	 * 
	 * effort = ((1-beta)/2)*surge_factor*_pid_surge + beta*yaw_factor_pid_yaw
	 *          + ((1-beta)/2)*sway_factor*_pid_sway
	 * 
	 * @param _pid_surge Ouput of surge PID/manual surge effort
	 * @param _pid_yaw Output of yaw PID
	 * @param _pid_sway Output of sway PID/manual sway effort
	 */
	void vectoredPid2Effort(double _pid_yaw, double _pid_surge, double _pid_sway) override;

	/**
	 * @brief Publishes the commands
	 */ 
	void sendCommands(void) override;

	/**
	 * @brief Stops all the thrusters.
	 */
	void allThrustersStop(void) override;
	
	/**
	 * @brief Manually set the effort values to the thrusters.
	 * 
	 * @param _manual_effort Pointer to the first element of the array containing the
	 * effort values for each thruster.
	 */
	void setEffort(double *_manual_effort) override;
};




#endif