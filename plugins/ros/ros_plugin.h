// File: plugins/ros/ros_plugin.h

#ifndef POLLIBEE_ROS_PLUGIN_H
#define POLLIBEE_ROS_PLUGIN_H

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>

namespace ros
{
  class RosPlugin : public ros::Node
  {
    public: RosPlugin();
    public: virtual ~RosPlugin();

    private: ros::NodeHandle nodeHandle;
    private: ros::Publisher imagePub;
    private: ros::Subscriber cameraSub;

    private: void cameraCallback(const sensor_msgs::Image::ConstPtr &_msg);
    private: void publishImage(const sensor_msgs::Image &_msg);
  };
}

#endif // POLLIBEE_ROS_PLUGIN_H
