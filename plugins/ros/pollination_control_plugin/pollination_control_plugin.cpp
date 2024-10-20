#include "pollination_control_plugin.h"

PollinationControlPlugin::PollinationControlPlugin() : nh_(nullptr),
  pollinationPub_(""), poseSub_(""), flowerDetectionSub_(""), batterySub_(""),
  windSpeedSub_(""), windDirectionSub_(""), currentPose_(), pollinationStatus_(),
  flowerPositions_(), windSpeed_(0.0), windDirection_(0.0, 0.0, 0.0), batteryLevel_(0.0),
  isPollinating_(false) {}

void PollinationControlPlugin::initialize(ros::NodeHandle& nh)
{
  nh_ = &nh;
  pollinationPub_ = nh_->advertise<std_msgs::Bool>("pollination_status", 10);
  poseSub_ = nh_->subscribe("drone_pose", 10, &PollinationControlPlugin::poseCallback, this);
  flowerDetectionSub_ = nh_->subscribe("flower_detection", 10, &PollinationControlPlugin::flowerDetectionCallback, this);
  batterySub_ = nh_->subscribe("battery_status", 10, &PollinationControlPlugin::batteryCallback, this);
  windSpeedSub_ = nh_->subscribe("wind_speed", 10, &PollinationControlPlugin::windSpeedCallback, this);
  windDirectionSub_ = nh_->subscribe("wind_direction", 10, &PollinationControlPlugin::windDirectionCallback, this);
}

void PollinationControlPlugin::controlPollination()
{
  if (isPollinating_)
  {
    stopPollination();
  }
  else
  {
    pollinate();
  }
}

void PollinationControlPlugin::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  currentPose_ = *msg;
}

void PollinationControlPlugin::flowerDetectionCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data)
  {
    pollinate();
  }
}

void PollinationControlPlugin::batteryCallback(const std_msgs::Float32::ConstPtr& msg)
{
  batteryLevel_ = msg->data;
  if (batteryLevel_ < 20.0)
  {
    stopPollination();
  }
}

void PollinationControlPlugin::windSpeedCallback(const std_msgs::Float32::ConstPtr& msg)
{
  windSpeed_ = msg->data;
}

void PollinationControlPlugin::windDirectionCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  windDirection_ = ignition::math::Vector3d(msg->x, msg->y, msg->z);
}

void PollinationControlPlugin::pollinate()
{
  if (!isPollinating_)
  {
    isPollinating_ = true;
    pollinationStatus_.data = true;
    pollinationPub_.publish(pollinationStatus_);
    ROS_INFO("Pollination started");
  }
}

void PollinationControlPlugin::stopPollination()
{
  if (isPollinating_)
  {
    isPollinating_ = false;
    pollinationStatus_.data = false;
    pollinationPub_.publish(pollinationStatus_);
    ROS_INFO("Pollination stopped");
  }
}

void PollinationControlPlugin::adjustForWind()
{
  // Implementați ajustarea pentru vânt
}
