// File: plugins/yolo/yolo_plugin.cpp

#include "yolo_plugin.h"

namespace yolo
{
  YoloPlugin::YoloPlugin() : ros::Node("yolo_plugin")
  {
    this->imageTransport = new image_transport::ImageTransport(this->nodeHandle);
    this->imagePub = this->imageTransport->advertise("yolo_image", 1);
    this->cameraSub = this->nodeHandle.subscribe("camera_image", 1, &YoloPlugin::cameraCallback, this);
  }

  YoloPlugin::~YoloPlugin()
  {
    delete this->imageTransport;
  }

  void YoloPlugin::cameraCallback(const sensor_msgs::Image::ConstPtr &_msg)
  {
    cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat detectedImage = runYolo(cvPtr->image);
    sensor_msgs::Image::Ptr detectedImageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", detectedImage).toImageMsg();
    this->imagePub.publish(detectedImageMsg);
  }

  cv::Mat YoloPlugin::runYolo(const cv::Mat &_image)
  {
    // Implementează algoritmul YOLO pentru detectarea obiectelor
    return _image; // placeholder
  }
}
