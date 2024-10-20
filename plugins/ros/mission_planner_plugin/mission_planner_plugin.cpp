#include "mission_planner_plugin.h"

MissionPlannerPlugin::MissionPlannerPlugin() : currentWaypointIndex(0),
  waypointReachedThreshold(0.5), batteryLevel(100.0), missionCompleted(false) {}

void MissionPlannerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  this->world = _model->GetWorld();

  if (_sdf->HasElement("waypointReachedThreshold"))
    this->waypointReachedThreshold = _sdf->Get<double>("waypointReachedThreshold");

  this->rosNode.reset(new ros::NodeHandle("mission_planner_plugin"));

  this->goalPub = this->rosNode->advertise<geometry_msgs::PoseStamped>("current_goal", 1000);
  this->pathPub = this->rosNode->advertise<nav_msgs::Path>("mission_path", 1000);
  this->batterySub = this->rosNode->subscribe("battery_status", 1000, &MissionPlannerPlugin::BatteryCallback, this);
  this->poseSub = this->rosNode->subscribe("drone_pose", 1000, &MissionPlannerPlugin::PoseCallback, this);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&MissionPlannerPlugin::OnUpdate, this));

  this->lastUpdateTime = this->world->SimTime();

  this->GenerateSurveyPath();

  ROS_INFO("Mission Planner plugin initialized");
}

void MissionPlannerPlugin::OnUpdate()
{
  common::Time currentTime = this->world->SimTime();
  double dt = (currentTime - this->lastUpdateTime).Double();

  if (dt > 0.1)
  {
    this->UpdateMission();
    this->PublishCurrentGoal();
    this->lastUpdateTime = currentTime;
  }
}

void MissionPlannerPlugin::GenerateSurveyPath()
{
  double fieldLength = 100.0;
  double fieldWidth = 50.0;
  double altitude = 10.0;
  double spacing = 10.0;

  for (double x = 0; x <= fieldLength; x += spacing)
  {
    if (static_cast<int>(x / spacing) % 2 == 0)
    {
      this->waypoints.push_back(ignition::math::Vector3d(x, 0, altitude));
      this->waypoints.push_back(ignition::math::Vector3d(x, fieldWidth, altitude));
    }
    else
    {
      this->waypoints.push_back(ignition::math::Vector3d(x, fieldWidth, altitude));
      this->waypoints.push_back(ignition::math::Vector3d(x, 0, altitude));
    }
  }

  nav_msgs::Path pathMsg;
  pathMsg.header.stamp = ros::Time::now();
  pathMsg.header.frame_id = "world";

  for (const auto& waypoint : this->waypoints)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = waypoint.X();
    pose.pose.position.y = waypoint.Y();
    pose.pose.position.z = waypoint.Z();
    pathMsg.poses.push_back(pose);
  }

  this->pathPub.publish(pathMsg);
}

void MissionPlannerPlugin::UpdateMission()
{
  if (this->missionCompleted)
    return;

  if (this->currentWaypointIndex >= this->waypoints.size())
  {
    ROS_INFO("Mission completed!");
    this->missionCompleted = true;
    return;
  }

  ignition::math::Vector3d currentWaypoint = this->waypoints[this->currentWaypointIndex];
  double distance = (this->currentPosition - currentWaypoint).Length();

  if (distance < this->waypointReachedThreshold)
  {
    ROS_INFO("Reached waypoint %zu", this->currentWaypointIndex);
    this->currentWaypointIndex++;
  }

  if (this->batteryLevel < 20.0 && this->currentWaypointIndex > 0)
  {
    ROS_WARN("Low battery! Returning to base.");
    this->waypoints.insert(this->waypoints.begin() + this->currentWaypointIndex,
                           ignition::math::Vector3d(0, 0, 10));
    this->currentWaypointIndex--;
  }
}

void MissionPlannerPlugin::PublishCurrentGoal()
{
  if (this->currentWaypointIndex < this->waypoints.size())
  {
    geometry_msgs::PoseStamped goalMsg;
    goalMsg.header.stamp = ros::Time::now();
    goalMsg.header.frame_id = "world";
    goalMsg.pose.position.x = this->waypoints[this->currentWaypointIndex].X();
    goalMsg.pose.position.y = this->waypoints[this->currentWaypointIndex].Y();
    goalMsg.pose.position.z = this->waypoints[this-> currentWaypointIndex].Z();
    this->goalPub.publish(goalMsg);
  }
}

void MissionPlannerPlugin::BatteryCallback(const std_msgs::Float32::ConstPtr& msg)
{
  this->batteryLevel = msg->data;
}

void MissionPlannerPlugin::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  this->currentPosition.Set(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}
