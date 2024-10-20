#include "wind_plugin.h"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/WindSensor.hh>

namespace gazebo
{
  GZ_REGISTER_MODEL_PLUGIN(WindPlugin)

  WindPlugin::WindPlugin() : windSpeed(0), windNoiseAmplitude(0.1),
    windGustProbability(0.01), windGustDuration(2.0), normalDistribution(0.0, 1.0) {}

  void WindPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    this->model = _model;
    this->world = _model->GetWorld();

    if (_sdf->HasElement("windNoiseAmplitude"))
      this->windNoiseAmplitude = _sdf->Get<double>("windNoiseAmplitude");
    if (_sdf->HasElement("windGustProbability"))
      this->windGustProbability = _sdf->Get<double>("windG ustProbability");
    if (_sdf->HasElement("windGustDuration"))
      this->windGustDuration = _sdf->Get<double>("windGustDuration");

    this->rosNode.reset(new ros::NodeHandle("wind_plugin"));

    this->windPub = this->rosNode->advertise<geometry_msgs::Vector3>("wind", 1000);
    this->windSpeedSub = this->rosNode->subscribe("wind_speed", 1000, &WindPlugin::WindSpeedCallback, this);
    this->windDirectionSub = this->rosNode->subscribe("wind_direction", 1000, &WindPlugin::WindDirectionCallback, this);

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&WindPlugin::OnUpdate, this));

    this->lastUpdateTime = this->world->SimTime();
    this->lastGustTime = this->world->SimTime();

    ROS_INFO("Wind plugin initialized");
  }

  void WindPlugin::OnUpdate()
  {
    common::Time currentTime = this->world->SimTime();
    double dt = (currentTime - this->lastUpdateTime).Double();

    this->UpdateWind(dt);
    this->ApplyWindForce();

    geometry_msgs::Vector3 windMsg;
    windMsg.x = this->windForce.X();
    windMsg.y = this->windForce.Y();
    windMsg.z = this->windForce.Z();
    this->windPub.publish(windMsg);
  }

  void WindPlugin::UpdateWind(double dt)
  {
    double noise = this->windNoiseAmplitude * this->normalDistribution(this->randomGenerator);
    this->windSpeed += noise;
    this->windSpeed = std::max(0.0, this->windSpeed);

    if (this->randomGenerator() / (double)this->randomGenerator.max() < this->windGustProbability * dt)
    {
      this->SimulateWindGust();
    }

    ignition::math::Vector3d directionNoise(
        this->normalDistribution(this->randomGenerator),
        this->normalDistribution(this->randomGenerator),
        0);
    this->windDirection += directionNoise * 0.1;
    this->windDirection.Normalize();

    this->windForce = this->windDirection * this->windSpeed;

    this->lastUpdateTime = currentTime;
  }

  void WindPlugin::WindSpeedCallback(const std_msgs::Float32::ConstPtr& msg)
  {
    this->windSpeed = msg->data;
  }

  void WindPlugin::WindDirectionCallback(const geometry_msgs::Vector3::ConstPtr& msg)
  {
    this->windDirection = ignition::math::Vector3d(msg->x, msg->y, msg->z);
    this->windDirection.Normalize();
  }

  void WindPlugin::ApplyWindForce()
  {
    physics::LinkPtr link = this->model->GetLink();
    if (link)
    {
      double airDensity = 1.225;
      double dragCoefficient = 0.47;
      double area = 0.1;

      ignition::math::Vector3d relativeWindVelocity = this->windForce - link->WorldLinearVel();
      double dynamicPressure = 0.5 * airDensity * relativeWindVelocity.SquaredLength();

      ignition::math::Vector3d dragForce = dragCoefficient * area * dynamicPressure * relativeWindVelocity.Normalized();

      link->AddForce(dragForce);
    }
  }

  void WindPlugin::SimulateWindGust()
  {
    common::Time currentTime = this->world->SimTime();
    double dt = (currentTime - this->lastGustTime).Double();

    if (dt > this->windGustDuration)
    {
      this->lastGustTime = currentTime;
      double gustMultiplier = 2.0 + this->normalDistribution(this->randomGenerator);
      this->windSpeed *= gustMultiplier;
    }
  }
}
