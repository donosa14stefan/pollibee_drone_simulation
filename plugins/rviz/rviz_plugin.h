// File: plugins/rviz/rviz_plugin.h

#ifndef POLLIBEE_RVIZ_PLUGIN_H
#define POLLIBEE_RVIZ_PLUGIN_H

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
  class RvizPlugin : public rviz::Display
  {
    public: RvizPlugin();
    public: virtual ~RvizPlugin();

    private: rviz::VisualizationManager* manager;
    private: rviz::RenderPanel* renderPanel;
    private: rviz::DisplayGroup* displayGroup;
    private: rviz::FrameManager* frameManager;

    private: void onInitialize();
    private: void onEnable();
    private: void onDisable();
    private: void processMessage(const sensor_msgs::Image::ConstPtr &_msg);
  };
}

#endif // POLLIBEE_RVIZ_PLUGIN_H
