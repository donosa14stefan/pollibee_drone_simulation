#ifndef WIND_PLUGIN_HH
#define WIND_PLUGIN_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <random>

namespace gazebo
{
  class WindPlugin : public ModelPlugin
  {
    public:
      WindPlugin();
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private:
      void OnUpdate(const common::UpdateInfo &_info);
      void WindSpeedCallback(const std_msgs::Float32::ConstPtr& msg);
      void WindDirectionCallback(const geometry_msgs::Vector3::ConstPtr& msg);
      void UpdateWind();
      void ApplyWindForce();
      void SimulateWindGust();

      physics::ModelPtr model;
      physics::WorldPtr world;
      event::ConnectionPtr updateConnection;
      
      std::unique_ptr<ros::NodeHandle> rosNode;
      ros::Publisher windPub;
      ros::Subscriber windSpeedSub;
      ros::Subscriber windDirectionSub;
      
      ignition::math::Vector3d windForce;
      double windSpeed;
      ignition::math::Vector3d windDirection;
      
      double windNoiseAmplitude;
      double windGustProbability;
      double windGustDuration;
      common::Time lastGustTime;
      common::Time lastUpdateTime;

      std::default_random_engine randomGenerator;
      std::normal_distribution<double> normalDistribution;
  };
}
#endif
