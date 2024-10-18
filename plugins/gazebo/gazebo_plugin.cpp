// File: plugins/gazebo/gazebo_plugin.cpp

#include "gazebo_plugin.h"

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
    this->link = _model-> GetLink();
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

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&PolliBeeDronePlugin::OnUpdate, this));
  }

  void PolliBeeDronePlugin::OnUpdate()
  {
    common::Time currentTime = this->model->GetWorld()->GetSimTime();
    double dt = (currentTime - this->lastUpdateTime).Double();

    ignition::math::Vector3d velocity = this->velocity;
    ignition::math::Vector3d acceleration = ignition::math::Vector3d(0, 0, 0);

    // Adaugă zgomotul de mișcare
    double smallNoise = this->standard_normal_distribution(this->random_generator) * this->motionSmallNoise;
    double driftNoise = this->standard_normal_distribution(this->random_generator) * this->motionDriftNoise;
    velocity += ignition::math::Vector3d(smallNoise, smallNoise, smallNoise);
    acceleration += ignition::math::Vector3d(driftNoise, driftNoise, driftNoise);

    // Limită viteza și accelerația
    velocity = ignition::math::Vector3d(std::min(std::max(velocity.X(), -this->maxForce), this->maxForce),
                                        std::min(std::max(velocity.Y(), -this->maxForce), this->maxForce),
                                        std::min(std::max(velocity.Z(), -this->maxForce), this->maxForce));
    acceleration = ignition::math::Vector3d(std::min(std::max(acceleration.X(), -this->maxForce), this->maxForce),
                                            std::min(std::max(acceleration.Y(), -this->maxForce), this->maxForce),
                                            std::min(std::max(acceleration.Z(), -this->maxForce), this->maxForce));

    // Actualizează starea modelului
    this->model->SetLinearVel(velocity);
    this->model->SetAngularVel(acceleration);

    // Publică poziția și viteza modelului
    msgs::Pose poseMsg;
    poseMsg.set_name(this->model->GetName());
    poseMsg.set_id(this->model->GetId());
    poseMsg.set_position(this->model->GetWorldPose().Pos());
    poseMsg.set_orientation(this->model->GetWorldPose().Rot());
    this->pubPose->Publish(poseMsg);

    this->lastUpdateTime = currentTime;
  }

  void PolliBeeDronePlugin::OnRosMsg(const geometry_msgs::TwistConstPtr &_msg)
  {
    this->velocity = ignition::math::Vector3d(_msg->linear.x, _msg->linear.y, _msg->linear.z);
  }

  void PolliBeeDronePlugin::OnCameraMsg(ConstImageStampedPtr &_msg)
  {
    sensor_msgs::Image imageMsg;
    imageMsg.header = _msg->time();
    imageMsg.height = _msg->image().height();
    imageMsg.width = _msg->image().width();
    imageMsg.step = _msg->image().step();
    imageMsg.data = _msg->image().data();
    this->cameraPub.publish(imageMsg);
  }
}
