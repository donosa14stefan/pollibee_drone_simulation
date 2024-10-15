#ifndef PYTHON_PLUGIN_H
#define PYTHON_PLUGIN_H

#include <ros/ros.h>
#include <ros/names.h>
#include <boost/python.hpp>

namespace python {
class PythonPlugin : public ros::Node {
public:
  PythonPlugin();
  virtual ~PythonPlugin();

private:
  ros::NodeHandle node_handle_;
  boost::thread python_thread_;

  void runPython();
};

PLUGINLIB_EXPORT_CLASS(python::PythonPlugin, ros::Node)

#endif // PYTHON_PLUGIN_H
