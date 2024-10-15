#ifndef YOLO_PLUGIN_H
#define YOLO_PLUGIN_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace yolo {
class YoloPlugin : public ros::Node {
public:
  YoloPlugin();
  virtual ~YoloPlugin();

  void cameraCallback(const sensor_msgs::Image::ConstPtr& msg);

private:
  ros::NodeHandle node_handle_;
  image_transport::ImageTransport* image_transport_;
  image_transport::Publisher image_pub_;
  ros::Subscriber camera_sub_;

  cv::Mat runYolo(const cv::Mat& image);
};

PLUGINLIB_EXPORT_CLASS(yolo::YoloPlugin, ros::Node)

#endif // YOLO_PLUGIN_H
