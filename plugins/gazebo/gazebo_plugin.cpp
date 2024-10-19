#include "gazebo_plugin.h"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>

namespace gazebo
{
  GZ_REGISTER_MODEL_PLUGIN(PolliBeeDronePlugin)

  PolliBeeDronePlugin::PolliBeeDronePlugin() : ModelPlugin(),
    maxForce(5.0),
    motionSmallNoise(0.05),
    motionDriftNoise(0.03),
    motionDriftNoiseTime(5.0)
  {
  }

  PolliBeeDronePlugin::~PolliBeeDronePlugin()
  {
    this->updateConnection.reset();
    this->rosNode->shutdown();
  }

  void PolliBeeDronePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    this->model = _model;
    this->link = _model->GetLink();
    this->joint = _model->GetJoint();
    this->cameraSensor = _model->GetSensor("camera");

    this->robotNamespace = _sdf->GetAttribute("robotNamespace")->GetAsString();
    this->topicName = _sdf->GetAttribute("topicName")->GetAsString();
    this->cameraTopicName = _sdf->GetAttribute("cameraTopicName")->GetAsString();

    this->rosNode.reset(new ros::NodeHandle(this->robotNamespace));
    this->rosPub = this->rosNode->advertise<geometry_msgs::Twist>(this->topicName, 10);
    this->rosSub = this->rosNode->subscribe(this->topicName, 10, &PolliBeeDronePlugin::OnRosMsg, this);
    this->cameraPub = this->rosNode->advertise<sensor_msgs::Image>(this->cameraTopicName, 10);

    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->robotNamespace);
    this->pubPose = this->node->Advertise<msgs::Pose>("~/pose", 10);
    this->subCamera = this->node->Subscribe("~/camera/image_raw", &PolliBeeDronePlugin::OnCameraMsg, this);

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&PolliBeeDronePlugin::OnUpdate, this));
  }

  void PolliBeeDronePlugin::OnUpdate()
  {
    common::Time currentTime = this->model->GetWorld()->GetSimTime();
    double dt = (currentTime - this->lastUpdateTime).Double();

    ignition::math::Vector3d velocity = this->velocity;
    ignition::math::Vector3d acceleration = ignition::math::Vector3d(0, 0, 0);

    velocity += ignition::math::Vector3d(this->motionSmallNoise * this->randomGenerator(),
                                         this->motionSmallNoise * this->randomGenerator(),
                                         this->motionSmallNoise * this->randomGenerator());
    acceleration += ignition::math::Vector3d(this->motionDriftNoise * this->randomGenerator(),
                                             this->motionDriftNoise * this->randomGenerator(),
                                             this->motionDriftNoise * this->randomGenerator());

    velocity = ignition::math::Vector3d(std::min(std::max(velocity.X(), -this->maxForce), this->maxForce),
                                        std::min(std::max(velocity.Y(), -this->maxForce), this->maxForce),
                                        std::min(std::max(velocity.Z(), -this->maxForce), this->maxForce));
    acceleration = ignition::math::Vector3d(std::min(std::max(acceleration.X(), -this->maxForce), this->maxForce),
                                            std::min(std::max(acceleration.Y(), -this->maxForce), this->maxForce),
                                            std::min(std::max(acceleration.Z(), -this->maxForce), this->maxForce));

    this->model->SetLinearVel(velocity);
    this->model->SetAngularVel(acceleration);

    msgs::Pose poseMsg;
    poseMsg.set_name(this->model->GetName());
    poseMsg.set_id(this->model->GetId());
    poseMsg.set_position(this->model->GetWorldPose().Pos());
    poseMsg.set_orientation(this->model->GetWorldPose().Rot());
    this->pubPose->Publish(p oseMsg);

    this->lastUpdateTime = currentTime;
  }

  void PolliBeeDronePlugin::OnRosMsg(const geometry_msgs::Twist::ConstPtr& msg)
  {
    this->velocity = ignition::math::Vector3d(msg->linear.x, msg->linear.y, msg->linear.z);
  }

  void PolliBeeDronePlugin::OnCameraMsg(ConstImageStampedPtr& msg)
  {
    sensor_msgs::Image imageMsg;
    imageMsg.header = msg->time();
    imageMsg.height = msg->image().height();
    imageMsg.width = msg->image().width();
    imageMsg.step = msg->image().step();
    imageMsg.data = msg->image().data();
    this->cameraPub.publish(imageMsg);
  }
}
