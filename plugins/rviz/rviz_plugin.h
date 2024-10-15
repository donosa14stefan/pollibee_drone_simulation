#ifndef RVIZ_PLUGIN_H
#define RVIZ_PLUGIN_H

#include <rviz/visualization_manager.hpp>
#include <rviz/render_panel.hpp>
#include <rviz/display.hpp>
#include <rviz/display_group.hpp>
#include <rviz/frame_manager.hpp>
#include <rviz/properties/property.hpp>
#include <rviz/properties/property_tree_model.hpp>
#include <rviz/properties/property_tree_widget.hpp>
#include <sensor_msgs/Image.h>

namespace rviz {
class RvizPlugin : public rviz::Display {
public:
  RvizPlugin();
  virtual ~RvizPlugin();

  void onInitialize();
  void onEnable();
  void onDisable();
  void processMessage(const sensor_msgs::Image::ConstPtr& msg);

private:
  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::DisplayGroup* display_group_;
  rviz::FrameManager* frame_manager_;
};

PLUGINLIB_EXPORT_CLASS(rviz::RvizPlugin, rviz::Display)

#endif // RVIZ_PLUGIN_H
