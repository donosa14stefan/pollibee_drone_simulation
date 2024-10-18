#ifndef POLLINATION_CONTROL_PLUGIN_HH
#define POLLINATION_CONTROL_PLUGIN_HH

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

class PollinationControlPlugin
{
  public:
    PollinationControlPlugin();
    void initialize(ros::NodeHandle& nh);
    void controlPollination();

  private:
    ros::NodeHandle* nh_;
    ros::Publisher pollinationPub_;
    ros::Subscriber poseSub_;
    ros::Subscriber flowerDetectionSub_;
    ros::Subscriber batterySub_;
    ros::Subscriber windSpeedSub_;
    ros::Subscriber windDirectionSub_;
    
    geometry_msgs::PoseStamped currentPose_;
    std_msgs::Bool pollinationStatus_;
    std::vector<geometry_msgs::PoseStamped> flowerPositions_;
    double windSpeed_;
    ignition::math::Vector3d windDirection_;
    double batteryLevel_;
    bool isPollinating_;
    
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void flowerDetectionCallback(const std_msgs::Bool::ConstPtr& msg);
    void batteryCallback(const std_msgs::Float32::ConstPtr& msg);
    void windSpeedCallback(const std_msgs::Float32::ConstPtr& msg);
    void windDirectionCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void pollinate();
    void stopPollination();
    void adjustForWind();
};
#endif
