#ifndef PLUGIN_DEPTH_H
#define PLUGIN_DEPTH_H

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#if (GAZEBO_MAJOR_VERSION == 11)
  #include <ignition/math6/ignition/math.hh>
#elif (GAZEBO_MAJOR_VERSION == 9)
  #include <ignition/math4/ignition/math.hh>
#endif


#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Empty.h>

namespace gazebo
{
class DepthSensor : public ModelPlugin
{
public:
  DepthSensor();
  virtual ~DepthSensor();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Reset();
  virtual void Update();

private:
  physics::WorldPtr world_;

  physics::LinkPtr link_;

  ros::NodeHandle* node_handle_;

  ros::CallbackQueue callback_queue_;
  ros::Publisher depth_publisher_;

  geometry_msgs::PointStamped depth_;

  std::string depth_topic_;
  std::string link_name_;

  double noise_;  // std dev of additive Gaussian noise

  event::ConnectionPtr updateConnection;
};
}

#endif  // PLUGIN_DEPTH_H