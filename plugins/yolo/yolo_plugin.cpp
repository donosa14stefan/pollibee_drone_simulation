#include "yolo_plugin.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace yolo {
class YoloPlugin : public ros::Node {
public:
  YoloPlugin() : ros::Node("yolo_plugin") {
    // Initialize ROS node
    this->image_transport_ = new image_transport::ImageTransport(this->node_handle_);
    this->image_pub_ = this->image_transport_->advertise("yolo_image", 1);
    this->camera_sub_ = this->node_handle_.subscribe("camera_image", 1, &YoloPlugin::cameraCallback, this);
  }

  ~YoloPlugin() {
    // Clean up
    delete this->image_transport_;
  }

  void cameraCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // Convert image message to OpenCV Mat
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // Run YOLO object detection on the image
    cv::Mat detected_image = runYolo(cv_ptr->image);

    // Publish detected image
    sensor_msgs::Image::Ptr detected_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", detected_image).toImageMsg();
    this->image_pub_.publish(detected_image_msg);
  }

private:
  ros::NodeHandle node_handle_;
  image_transport::ImageTransport* image_transport_;
  image_transport::Publisher image_pub_;
  ros::Subscriber camera_sub_;

  cv::Mat runYolo(const cv::Mat& image) {
    // TO DO: implement YOLO object detection algorithm
    return image; // placeholder
  }
};

PLUGINLIB_EXPORT_CLASS(yolo::YoloPlugin, ros::Node)
}
