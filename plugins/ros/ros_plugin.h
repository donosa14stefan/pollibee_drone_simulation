#ifndef ROS_PLUGIN_H
#define ROS_PLUGIN_H

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>

namespace ros {
class RosPlugin : public ros::Node {
public:
  RosPlugin();
  virtual ~RosPlugin();

  void cameraCallback(const sensor_msgs::Image::ConstPtr& msg);
  void publishImage(const sensor_msgs::Image& image);

private:
  ros::NodeHandle node_handle_;
  ros::Publisher image_pub_;
  ros::Subscriber camera_sub_;
};

PLUGINLIB_EXPORT_CLASS(ros::RosPlugin, ros::Node)

#endif // ROS_PLUGIN_H
