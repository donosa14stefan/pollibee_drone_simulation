#ifndef MISSION_PLANNER_PLUGIN_HH
#define MISSION_PLANNER_PLUGIN_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <vector>

namespace gazebo
{
  class MissionPlannerPlugin : public ModelPlugin
  {
    public:
      MissionPlannerPlugin();
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private:
      void OnUpdate(const common::UpdateInfo &_info);
      void GenerateSurveyPath();
      void UpdateMission();
      void PublishCurrentGoal();
      void BatteryCallback(const std_msgs::Float32::ConstPtr& msg);
      void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

      physics::ModelPtr model;
      physics::WorldPtr world;
      event::ConnectionPtr updateConnection;
      
      std::unique_ptr<ros::NodeHandle> rosNode;
      ros::Publisher goalPub;
      ros::Publisher pathPub;
      ros::Subscriber batterySub;
      ros::Subscriber poseSub;
      
      std::vector<ignition::math::Vector3d> waypoints;
      size_t currentWaypointIndex;
      double waypointReachedThreshold;
      double batteryLevel;
      ignition::math::Vector3d currentPosition;
      bool missionCompleted;

      common::Time lastUpdateTime;
  };
}
#endif
