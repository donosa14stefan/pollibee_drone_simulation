#include "python_plugin.h"
#include <ros/ros.h>
#include <ros/python_init.h>
#include <ros/names.h>
#include <boost/python.hpp>

namespace python {
class PythonPlugin : public ros::Node {
public:
  PythonPlugin() : ros::Node("python_plugin") {
    // Initialize ROS node
    this->node_handle_ = ros::NodeHandle("~");
    this->python_thread_ = boost::thread(boost::bind(&PythonPlugin::runPython, this));
  }

  ~PythonPlugin() {
    // Clean up
    this->python_thread_.join();
  }

private:
  ros::NodeHandle node_handle_;
  boost::thread python_thread_;

  void runPython() {
    // Initialize Python interpreter
    Py_Initialize();
    PyEval_InitThreads();

    // Import Python module
    boost::python::object module = boost::python::import("python_module");

    // Run Python code
    module.attr("run")();

    // Clean up
    Py_Finalize();
  }
};

PLUGINLIB_EXPORT_CLASS(python::PythonPlugin, ros::Node)
}
