#ifndef BATTERY_PLUGIN_HH
#define BATTERY_PLUGIN_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
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

    private:
      void OnUpdate(const common::UpdateInfo &_info);
      void VelocityCallback(const geometry_msgs::Twist::ConstPtr& msg);
      void UpdateBatteryStatus();
      void PublishBatteryStatus();
      void CalculateDischargeRate(const geometry_msgs::Twist& velocity);

      physics::ModelPtr model;
      physics::WorldPtr world;
      event::ConnectionPtr updateConnection;
      
      std::unique_ptr<ros::NodeHandle> rosNode;
      ros::Publisher batteryPub;
      ros::Subscriber velocitySub;
      
      double batteryCapacity;
      double currentCharge;
      double dischargeRate;
      double chargingRate;
      double lowBatteryThreshold;
      bool isCharging;

      common::Time lastUpdateTime;
  };
}
#endif
