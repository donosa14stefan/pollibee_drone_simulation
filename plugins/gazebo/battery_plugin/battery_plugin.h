#ifndef BATTERY_PLUGIN_HH
#define BATTERY_PLUGIN_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

namespace gazebo
{
  class BatteryPlugin : public ModelPlugin
  {
    public:
      BatteryPlugin();
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

    private:
      physics::ModelPtr model;
      event::ConnectionPtr updateConnection;
      ros::NodeHandle* rosNode;
      ros::Publisher batteryPub;
      ros::Subscriber velocitySub;
      
      double batteryCapacity;
      double currentCharge;
      double dischargeRate;
      double chargingRate;
      double lowBatteryThreshold;
      bool isCharging;
      
      void UpdateBatteryStatus();
      void VelocityCallback(const geometry_msgs::Twist::ConstPtr& msg);
      void CalculateDischargeRate(const geometry_msgs::Twist& velocity);
      void PublishBatteryStatus();
      bool NeedsCharging() const;
      void StartCharging();
      void StopCharging();
  };
}
#endif
