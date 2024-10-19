#include "rviz_plugin.h"
#include <ros/ros.h>
#include <rviz/visualization_manager.hpp>
#include <rviz/render_panel.hpp>
#include <rviz/display.hpp>
#include <rviz/display_group.hpp>
#include <rviz/frame_manager.hpp>
#include <rviz/properties/property.hpp>
#include <rviz/properties/property_tree_model.hpp>
#include <rviz/properties/property_tree_widget.hpp>
#include <sensor_msgs/Image.h>

namespace rviz
{
  RvizPlugin::RvizPlugin() : rviz::Display()
  {
    this->setName("RvizPlugin");
    this->setIcon(QIcon(":/icons/rviz_plugin.png"));
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

  void RvizPlugin::processMessage(const sensor_msgs::Image::ConstPtr& msg)
  {
    // Procesați imaginea de la camera
  }
}
