// File: plugins/rviz/rviz_plugin.cpp

#include "rviz_plugin.h"

namespace rviz
{
  RvizPlugin::RvizPlugin() : rviz::Display()
  {
    this->setName("RvizPlugin");
    this->setIcon(QIcon(":/icons/rviz_plugin.png"));
  }

  RvizPlugin::~RvizPlugin()
  {
  }

  void RvizPlugin::onInitialize()
  {
    this->initializeManager();
  }

  void RvizPlugin::onEnable()
  {
    this->enableManager();
  }

  void RvizPlugin::onDisable()
  {
    this->disableManager();
  }

  void RvizPlugin::processMessage(const sensor_msgs::Image::ConstPtr &_msg)
  {
    // Procesează imaginea de la camera
  }
}
