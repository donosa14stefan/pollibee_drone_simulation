// File: plugins/yolo/yolo_plugin.cpp

#include "yolo_plugin.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace yolo
{
  YoloPlugin::YoloPlugin() : imageTransport(*this), running(true)
  {
    try {
      loadConfig();
      initializeYOLO();
      imagePub = imageTransport.advertise("yolo_image", 1);
      cameraSub = subscribe("camera_image", 1, &YoloPlugin::cameraCallback, this);
      processingThread = std::make_unique<std::thread>(&YoloPlugin::processImages, this);
      ROS_INFO("YOLOv11 plugin initialized successfully");
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("Error initializing YOLOv11 plugin: " << e.what());
      throw;
    }
  }

  void YoloPlugin::loadConfig()
  {
    nlohmann::json config;
    std::ifstream configFile("yolo_config.json");
    if (!configFile) {
      throw std::runtime_error("Unable to open YOLO configuration file");
    }
    configFile >> config;

    configPath = config["config_path"];
    weightsPath = config["weights_path"];
    classesPath = config["classes_path"];
    confThreshold = config["confidence_threshold"];
    nmsThreshold = config["n ms_threshold"];
    inpWidth = config["input_width"];
    inpHeight = config["input_height"];
  }

  void YoloPlugin::initializeYOLO()
  {
    net = cv::dnn::readNetFromDarknet(configPath, weightsPath);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

    std::ifstream classNamesFile(classesPath);
    if (!classNamesFile) {
      throw std::runtime_error("Unable to open YOLO classes file");
    }
    std::string line;
    while (std::getline(classNamesFile, line)) {
      classNames.push_back(line);
    }
  }

  void YoloPlugin::cameraCallback(const sensor_msgs::Image::ConstPtr &msg)
  {
    try {
      cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      {
        std::lock_guard<std::mutex> lock(mtx);
        imageQueue.push(cvPtr->image);
      }
      cv.notify_one();
    } catch (const cv_bridge::Exception& e) {
      ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    }
  }

  cv::Mat YoloPlugin::runYolo(const cv::Mat &image)
  {
    cv::Mat blob = cv::dnn::blobFromImage(image, 1 / 255.0, cv::Size(inpWidth, inpHeight), cv::Scalar(0, 0, 0), true, false);
    net.setInput(blob);
    std::vector<cv::Mat> outputs = net.forward(getOutputsNames(net));

    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (size_t i = 0; i < outputs.size(); ++i) {
      float* data = (outputs[i].data);
      for (int j = 0; j < outputs[i].rows; ++j, data += outputs[i].cols) {
        cv::Mat scores = outputs[i].row(j).colRange(5, outputs[i].cols);
        cv::Point classIdPoint;
        double confidence;
        cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
        if (confidence > confThreshold) {
          int centerX = (int)(data[0] * image.cols);
          int centerY = (int)(data[1] * image.rows);
          int width = (int)(data[2] * image.cols);
          int height = (int)(data[3] * image.rows);
          int left = centerX - width / 2;
          int top = centerY - height / 2;

          classIds.push_back(classIdPoint.x);
          confidences.push_back((float)confidence);
          boxes.push_back(cv::Rect(left, top, width, height));
        }
      }
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    for (size_t i = 0; i < indices.size(); ++i) {
      int idx = indices[i];
      cv::Rect box = boxes[idx];
      drawPred(classIds[idx], (float)confidences[idx], box.x, box.y, box.x + box.width, box.y + box.height, image);
    }

    return image;
  }

  void YoloPlugin::drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat &frame)
  {
    cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 255, 0), 2);
    std::string label = cv::format("%.2f", conf);
    if (!classNames.empty()) {
      CV_Assert(classId < (int)classNames.size());
      label = classNames[classId] + ":" + label;
    }
    cv::putText(frame, label, cv::Point(left, top - 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
  }

  std::vector<cv::Mat> YoloPlugin::getOutputsNames(const cv::dnn::Net &net)
  {
    std::vector<cv::Mat> names;
    for (int i = 0; i < net.getLayerCount(); ++i) {
      cv::String name = net.getLayer(i)->name;
      if (name.find("yolo") != cv::String::npos) {
        names.push_back(net.getLayer(i)->outputNameToIndex(name));
      }
    }
    return names;
  }

  void Yolo Plugin::processImages()
  {
    while (running) {
      cv::Mat image;
      {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this] { return !imageQueue.empty() || !running; });
        if (!running) {
          return;
        }
        image = imageQueue.front();
        imageQueue.pop();
      }
      cv::Mat detectedImage = runYolo(image);
      sensor_msgs::Image::Ptr detectedImageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", detectedImage).toImageMsg();
      imagePub.publish(detectedImageMsg);
    }
  }
}
