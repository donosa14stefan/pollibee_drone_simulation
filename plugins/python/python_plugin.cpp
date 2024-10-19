#include "python_plugin.h"
#include <ros/python_init.h>
#include <ros/names.h>
#include <boost/python.hpp>

namespace python
{
  PythonPlugin::PythonPlugin() : ros::Node("python_plugin")
  {
    this->nodeHandle = ros::NodeHandle("~");
    this->pythonThread = boost::thread(boost::bind(&PythonPlugin::runPython, this));
  }

  PythonPlugin::~PythonPlugin()
  {
    this->pythonThread.join();
  }

  void PythonPlugin::runPython()
  {
    Py_Initialize();
    PyEval_InitThreads();

    boost::python::object module = boost::python::import("python_module");

    module.attr("run")();

    Py_Finalize();
  }
}
