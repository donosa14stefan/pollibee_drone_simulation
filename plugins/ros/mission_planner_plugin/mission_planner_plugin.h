#ifndef MISSION_PLANNER_PLUGIN_HH
#define MISSION_PLANNER_PLUGIN_HH

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <vector>

class MissionPlannerPlugin
{
  public:
    MissionPlannerPlugin();
    void initialize(ros::NodeHandle& nh);
    void planMission();
    void updateMission();

  private:
    ros::NodeHandle* nh_;
    ros::Publisher pathPub_;
    ros::Subscriber poseSub_;
    ros::Subscriber flowerDetectionSub_;
    ros::Subscriber batterySub_;
    ros::Subscriber windSpeedSub_;
    ros::Subscriber windDirectionSub_;
    
    nav_msgs::Path currentPath_;
    geometry_msgs::PoseStamped currentPose_;
    std::vector<geometry_msgs::PoseStamped> flowerPositions_;
    std::vector<geometry_msgs::PoseStamped> surveyPoints_;
    double windSpeed_;
    ignition::math::Vector3d windDirection_;
    double batteryLevel_;
    bool isPollinating_;
    
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void flowerDetectionCallback(const std_msgs::Bool::ConstPtr& msg);
    void batteryCallback(const std_msgs::Float32::ConstPtr& msg);
    void windSpeedCallback(const std_msgs::Float32::ConstPtr& msg);
    void windDirectionCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void generateSurveyPoints();
    void generatePath();
    void updatePath();
    void publishPath();
};
#endif
