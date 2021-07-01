#ifndef VEC6_COMMS_H
#define VEC6_COMMS_H

#include <ros/ros.h>

// Misc libraries
#include "vec6_config.h"
#include <cmath>

// Message data-types
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#ifdef USE_DARKNET
#include "darknet_ros_msgs/BoundingBoxes.h"
#endif

/**
 * @brief Manages the communication between the various sub-systems of vec6.
 */
class Vec6Comms{
public:
	virtual ~Vec6Comms(){}

	// Message topics
	const std::string effort_topic_ = THRUSTER_EFFORT_TOPIC;
	const std::string depth_topic_ = DEPTH_TOPIC;
	const std::string imu_topic_ = IMU_TOPIC;
	#ifdef USE_DARKNET
	const std::string bounding_box_topic_ = BOUNDING_BOX_TOPIC;
	#endif

	/// @brief Pointer to the instance storing state of the vehicle
  	Vec6State *state_;

	/// @brief Thruster effort publisher
	ros::Publisher effort_pub_;

	/// @brief Depth data subscriber
	ros::Subscriber depth_sub_;

	/// @brief IMU data subscriber
	ros::Subscriber imu_sub_;

	#ifdef USE_DARKNET
	/// @brief Bounding box data subscriber
	ros::Subscriber bb_sub_;

	/// @brief Bounding boxes message
  	darknet_ros_msgs::BoundingBoxes bbxs_;
	#endif

	/**
	 * @brief Initializes the communication between the sub-systems of
	 * vec6 by creating publishers and subscribers.
	 * 
	 * @param node Reference to the node handle of the main ROS node
	 * @param _vec6state Reference to the instance storing the state of vec6
	 */
	void initComms(ros::NodeHandle& _node, Vec6State& _vec6state);

	/**
	 * @brief Callback function listening to depth sensor's output.
	 */
	void depthCallbck(const geometry_msgs::PointStamped&);

	/**
	 * @brief Callback function listening to filtered IMU data.
	 */
  	void imuCallbck(const sensor_msgs::Imu&);

	#ifdef USE_DARKNET
	/**
	 * @brief Callback function listening to darknet's bounding
	 * boxes output.
	 */
  	void bbCallbck(const darknet_ros_msgs::BoundingBoxes&);
	#endif

};



#endif