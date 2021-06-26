#include "plugin_depth.h"

#include <cmath>
#include <iostream>
#include <stdlib.h>

namespace gazebo
{
// Contructor
DepthSensor::DepthSensor()
{
}

// Destructor
DepthSensor::~DepthSensor()
{
  node_handle_->shutdown();
  delete node_handle_;
}

// Load the plugin
void DepthSensor::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world_ = _model->GetWorld();
  link_ = _model->GetLink();
  link_name_ = link_->GetName();

  if (_sdf->HasElement("bodyName"))
  {
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
    link_ = _model->GetLink(link_name_);
  }

  if (!link_)
  {
    gzerr << "Cannot load plugin_depth because bodyName: " << link_name_ << " does not exist." << std::endl;
    return;
  }

  // verifying the presence and loading parameter values from the URDF elements
  if (_sdf->HasElement("depthTopicName")){
    depth_topic_ = _sdf->GetElement("depthTopicName")->Get<std::string>();
  }
  else{
    gzerr << "Cannot load plugin-depth because depthTopicName element is not included.\n";
    return;
  }
  if (_sdf->HasElement("noise")){
    noise_ = _sdf->GetElement("noise")->Get<double>();
  }
  else{
    gzerr << "Cannot load plugin-depth because noise element is not included.\n";
    return;
  }
  depth_.header.frame_id = link_->GetName();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable "
                     "to load plugin. "
                     << "Load the Gazebo system plugin "
                        "'libgazebo_ros_api_plugin.so' in the gazebo_ros "
                        "package)");
    return;
  }

  node_handle_ = new ros::NodeHandle;

  // advertise depth
  if (!depth_topic_.empty())
  {
    depth_publisher_ = node_handle_->advertise<geometry_msgs::PointStamped>(depth_topic_, 1);
  }
  else{
    gzerr << "Cannot load plugin-depth because depthTopicName is empty.\n";
    return;
  }

  // connect Update function
  updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&DepthSensor::Update, this));

  gzmsg << "Loaded plugin-depth\n";
}

void DepthSensor::Reset()
{
}

// Update
void DepthSensor::Update()
{
  // Get new command
  callback_queue_.callAvailable();

#if (GAZEBO_MAJOR_VERSION >= 8)
  common::Time sim_time = world_->SimTime();
#else
  common::Time sim_time = world_->GetSimTime();
#endif

#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Pose3d pose = link_->WorldPose();
  double depth = pose.Pos().Z();
#else
  math::Pose pose = link_->GetWorldPose();
  double depth = pose.pos.z;
#endif

  // add noise to the actual value
  depth += noise_ * drand48();

  depth_.header.stamp = ros::Time::now();
  depth_.point.z = depth;
  depth_publisher_.publish(depth_);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DepthSensor)

} 
