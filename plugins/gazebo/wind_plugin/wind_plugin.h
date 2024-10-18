#ifndef WIND_PLUGIN_HH
#define WIND_PLUGIN_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>

namespace gazebo
{
  class WindPlugin : public ModelPlugin
  {
    public:
      WindPlugin();
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

    private:
      physics::ModelPtr model;
      event::ConnectionPtr updateConnection;
      ros::NodeHandle* rosNode;
      ros::Publisher windPub;
      ros::Subscriber windSpeedSub;
      ros::Subscriber windDirectionSub;
      
      double windSpeed;
      ignition::math::Vector3d windDirection;
      
      double windNoiseAmplitude;
      double windGustProbability;
      double windGustDuration;
      double lastGustTime;
      
      void UpdateWind();
      void WindSpeedCallback(const std_msgs::Float32::ConstPtr& msg);
      void WindDirectionCallback(const geometry_msgs::Vector3::ConstPtr& msg);
      void ApplyWindForce();
      void SimulateWindGust();
  };
}
#endif
