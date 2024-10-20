#ifndef POLLIBEE_YOLO_PLUGIN_H
#define POLLIBEE_YOLO_PLUGIN_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv 2/dnn.hpp>
#include <vector>
#include <string>
#include <memory>
#include <nlohmann/json.hpp>

namespace yolo
{
  class YoloPlugin : public ros::NodeHandle
  {
  public:
    YoloPlugin();
    virtual ~YoloPlugin() = default;

  private:
    image_transport::ImageTransport imageTransport;
    image_transport::Publisher imagePub;
    ros::Subscriber cameraSub;

    cv::dnn::Net net;
    std::vector<std::string> classNames;
    float confThreshold;
    float nmsThreshold;
    int inpWidth;
    int inpHeight;

    std::string configPath;
    std::string weightsPath;
    std::string classesPath;

    void loadConfig();
    void initializeYOLO();
    void cameraCallback(const sensor_msgs::Image::ConstPtr &msg);
    cv::Mat runYolo(const cv::Mat &image);
    void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);
    std::vector<cv::Mat> getOutputsNames(const cv::dnn::Net& net);

    // Multithreading
    std::unique_ptr<std::thread> processingThread;
    std::mutex mtx;
    std::condition_variable cv;
    std::queue<cv::Mat> imageQueue;
    bool running;

    void processImages();
  };
}
#endif
