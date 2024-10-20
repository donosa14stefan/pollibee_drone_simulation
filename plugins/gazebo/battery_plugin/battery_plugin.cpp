#include "battery_plugin.h"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>

namespace gazebo
{
  GZ_REGISTER_MODEL_PLUGIN(BatteryPlugin)

  BatteryPlugin::BatteryPlugin() : batteryCapacity(100), currentCharge(100),
    dischargeRate(0.1), chargingRate(1.0), lowBatteryThreshold(20), isCharging(false) {}

  void BatteryPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    this->model = _model;
    this->world = _model->GetWorld();

    if (_sdf->HasElement("batteryCapacity"))
      this->batteryCapacity = _sdf->Get<double>("batteryCapacity");
    if (_sdf->HasElement("dischargeRate"))
      this->dischargeRate = _sdf->Get<double>("dischargeRate");
    if (_sdf->HasElement("chargingRate"))
      this->chargingRate = _sdf->Get<double>("chargingRate");
    if (_sdf->HasElement("lowBatteryThreshold"))
      this->lowBatteryThreshold = _sdf->Get<double>("lowBatteryThreshold");

    this->rosNode.reset(new ros::NodeHandle("battery_plugin"));

    this->batteryPub = this->rosNode->advertise<std_msgs::Float32>("battery_status", 1000);
    this->velocitySub = this->rosNode->subscribe("cmd_vel", 1000, &BatteryPlugin::VelocityCallback, this);

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&BatteryPlugin::OnUpdate, this));

    this->lastUpdateTime = this->world->SimTime();

    ROS_INFO("Battery plugin initialized");
  }

  void BatteryPlugin::OnUpdate()
  {
    common::Time currentTime = this->world->SimTime();
    double dt = (currentTime - this->lastUpdateTime).Double();

    this->UpdateBatteryStatus(dt);
    this->PublishBatteryStatus();

    this->lastUpdateTime = currentTime;
  }

  void BatteryPlugin::UpdateBatteryStatus(double dt)
  {
    if (this->isCharging)
    {
      this->currentCharge = std::min(this->batteryCapacity, this->currentCharge + this->chargingRate * dt);
    }
    else
    {
      this->currentCharge = std::max(0.0, this->currentCharge - this->dischargeRate * dt);
    }

    if (this->currentCharge <= this->lowBatteryThreshold && !this->isCharging)
    {
      ROS_WARN("Low battery! Current charge: %.2f%%", this->currentCharge);
    }
  }

  void BatteryPlugin::VelocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    this->CalculateDischargeRate(*msg);
  }

  void BatteryPlugin::PublishBatteryStatus()
  {
    std_msgs::Float32 msg;
    msg.data = this->currentCharge;
    this->batteryPub.publish(msg);
  }

  void BatteryPlugin::CalculateDischargeRate(const geometry_msgs::Twist& velocity)
  {
    double linearVelocity = std::sqrt(velocity.linear.x * velocity.linear.x +
                                      velocity.linear.y * velocity.linear.y +
                                      velocity.linear.z * velocity.linear.z);
    double angularVelocity = std::abs(velocity.angular.z);

    double linearConsumptionFactor = 0.1;
    double angularConsumptionFactor = 0.05;

    this->dischargeRate = 0.05 + linearVelocity * linearConsumptionFactor +
                          angularVelocity * angularConsumptionFactor;
  }
}
