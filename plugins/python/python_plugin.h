// File: plugins/python/python_plugin.h

#ifndef POLLIBEE_PYTHON_PLUGIN_H
#define POLLIBEE_PYTHON_PLUGIN_H

#include <ros/ros.h>
#include <ros/names.h>
#include <boost/python.hpp>

namespace python
{
  class PythonPlugin : public ros::Node
  {
    public: PythonPlugin();
    public: virtual ~PythonPlugin();

    private: ros::NodeHandle nodeHandle;
    private: boost::thread pythonThread;

    private: void runPython();
  };
}

#endif // POLLIBEE_PYTHON_PLUGIN_H
