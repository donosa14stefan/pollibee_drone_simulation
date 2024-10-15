#ifndef GAZEBO_PLUGIN_H
#define GAZEBO_PLUGIN_H

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>

namespace gazebo {
class GazeboPlugin : public ModelPlugin {
public:
  GazeboPlugin();
  virtual ~GazeboPlugin();

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void Reset();
  void Fini();

private:
  void cameraCallback(const sensor_msgs::Image::ConstPtr& msg);
  void update();
  void OnWorldUpdateBegin();
  void OnCameraImage(const sensor_msgs::Image::ConstPtr& msg);
  void OnRosShutdown(const ros::ShutdownEventArgs& event);

  ros::NodeHandle node_handle_;
  ros::Publisher image_pub_;
  ros::Subscriber camera_sub_;
  physics::ModelPtr model_;
  sdf::ElementPtr sdf_;
  sensors::CameraSensorPtr camera_sensor_;
  event::ConnectionPtr update_connection_;
};

GZ_REGISTER_MODEL_PLUGIN(GazeboPlugin)

#endif // GAZEBO_PLUGIN_H
