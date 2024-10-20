#ifndef POLLIBEE_GAZEBO_PLUGIN_H
#define POLLIBEE_GAZEBO_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <memory>
#include <string>

namespace gazebo
{
  class PolliBeeDronePlugin : public ModelPlugin
  {
    public: PolliBeeDronePlugin();
    public: virtual ~PolliBeeDronePlugin();

    private: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    protected: virtual void OnUpdate();

    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Publisher rosPub;
    private: ros::Subscriber rosSub;
    private: ros::Publisher cameraPub;

    private: std::unique_ptr<transport::Node> node;
    private: transport::PublisherPtr pubPose;
    private: transport::SubscriberPtr subCamera;

    private: physics::ModelPtr model;
    private: physics::LinkPtr link;
    private: physics::JointPtr joint;
    private: sensors::CameraSensorPtr cameraSensor;

    private: event::ConnectionPtr updateConnection;

    private: std::string robotNamespace;
    private: std::string topicName;
    private: std::string cameraTopicName;

    private: double maxForce;
    private: double motionSmallNoise;
    private: double motionDriftNoise;
    private: double motionDriftNoiseTime;

    private: ignition::math::Vector3d velocity;
    private: common::Time lastUpdateTime;

    private: std::default_random_engine random_generator;
    private: std::normal_distribution<double> standard_normal_distribution;
  };
}
#endif
