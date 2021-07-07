#include "vec6_comms.h"

void Vec6Comms::initComms(ros::NodeHandle &_node, Vec6State& _vec6state){
	// Initialize publisher
	effort_pub_ = _node.advertise<sensor_msgs::JointState>(effort_topic_, 1);

  	// Initialize and check for subscription
	depth_sub_ = _node.subscribe(depth_topic_, 1, &Vec6Comms::depthCallbck, this);
  	if (depth_sub_){
    	ROS_INFO_STREAM("Subscribed to topic: " << depth_topic_);
  	}
  	else{
    	ROS_WARN_STREAM("Could not subscribe to topic: " << depth_topic_);
  	}

	imu_sub_ = _node.subscribe(imu_topic_, 1, &Vec6Comms::imuCallbck, this);
	if (imu_sub_){
		ROS_INFO_STREAM("Subscribed to topic: " << imu_topic_);
	}
	else{
		ROS_WARN_STREAM("Could not subscribe to topic: " << imu_topic_);
	}

	#ifdef USE_DARKNET
	bb_sub_ = _node.subscribe(bounding_box_topic_, 1, &Vec6Comms::bbCallbck, this);
	if (bb_sub_){
		ROS_INFO_STREAM("Subscribed to topic: " << bounding_box_topic_);
	}
	else{
		ROS_WARN_STREAM("Could not subscribe to topic: " << bounding_box_topic_);
	}
	#endif

	state_ = &_vec6state;
}

void Vec6Comms::depthCallbck(const geometry_msgs::PointStamped& _depth)
{
  	state_->cur_loc_.z = -_depth.point.z; 		// negating to conform to accepted convention
}

void Vec6Comms::imuCallbck(const sensor_msgs::Imu& imu_)
{
  double sqw = imu_.orientation.w * imu_.orientation.w;
  double sqx = imu_.orientation.x * imu_.orientation.x;
  double sqy = imu_.orientation.y * imu_.orientation.y;
  double sqz = imu_.orientation.z * imu_.orientation.z;
  
  state_->cur_orient_.roll = atan2(2.0 * (imu_.orientation.y * imu_.orientation.z + imu_.orientation.x * imu_.orientation.w),
                          (-sqx - sqy + sqz + sqw)) *
                    RAD2DEG;
  state_->cur_orient_.pitch = asin(2.0 * (imu_.orientation.y * imu_.orientation.w - imu_.orientation.x * imu_.orientation.z) /
                          (sqx + sqy + sqz + sqw)) *
                     RAD2DEG;
  state_->cur_orient_.yaw = -atan2(2.0 * (imu_.orientation.x * imu_.orientation.y + imu_.orientation.z * imu_.orientation.w),
                         (sqx - sqy - sqz + sqw)) *
                   RAD2DEG; // negating to conform to accepted convention
}

#ifdef USE_DARKNET
void Vec6Comms::bbCallbck(const darknet_ros_msgs::BoundingBoxes& _bb)
{
  bbxs_ = _bb;
}
#endif