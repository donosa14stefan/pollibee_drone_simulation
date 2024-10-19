#include "ros_plugin.h"

namespace ros
{
  RosPlugin::RosPlugin() : ros::Node("ros_plugin")
  {
    this->nodeHandle = ros::NodeHandle("~");
    this->imagePub = this->nodeHandle.advertise<sensor_msgs::Image>("image", 10);
    this->cameraSub = this->nodeHandle.subscribe("camera", 10, &RosPlugin::cameraCallback, this);
  }

  RosPlugin::~RosPlugin()
  {
  }

  void RosPlugin::cameraCallback(const sensor_msgs::Image::ConstPtr& msg)
  {
    // Procesați imaginea de la camera
  }

  void RosPlugin::publishImage(const sensor_msgs::Image& msg)
  {
    this->imagePub.publish(msg);
  }
}
