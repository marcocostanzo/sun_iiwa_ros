#ifndef SUN_IIWA_NODES_H_
#define SUN_IIWA_NODES_H_

#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointVelocity.h"
#include "sun_robot_lib/Robots/LBRiiwa7.h"
#include "sun_robot_ros/FkineNode.h"

namespace sun {

class IIWA_FkineNode : public FkineNode {
private:
  ros::Subscriber joint_sub_, jointvel_sub_;
public:
  static std::shared_ptr<sun::LBRiiwa7> makeIIWA() {
    return std::make_shared<sun::LBRiiwa7>("iiwa7");
  }

  IIWA_FkineNode(
      const ros::NodeHandle &nh_for_topics = ros::NodeHandle("clik"),
      const ros::NodeHandle &nh_for_parmas = ros::NodeHandle("~"),
      ros::CallbackQueue *callbk_queue = ros::getGlobalCallbackQueue())
      : FkineNode(IIWA_FkineNode::makeIIWA(), nh_for_topics, nh_for_parmas,
                  callbk_queue) {}

  ~IIWA_FkineNode() = default;

  /**
     joint_sub_ = ...
   */
  virtual void registerJointSubscriber() override {
    joint_sub_ = nh_.subscribe("/iiwa/state/JointPosition", 1,
                               &IIWA_FkineNode::joint_position_cb, this);
    jointvel_sub_ = nh_.subscribe("/iiwa/state/JointVelocity", 1,
                                  &IIWA_FkineNode::joint_vel_cb, this);
  }

  TooN::Vector<7> qR_ = TooN::Zeros;
  void
  joint_position_cb(const iiwa_msgs::JointPosition::ConstPtr &joi_state_msg) {
    qR_[0] = joi_state_msg->position.a1;
    qR_[1] = joi_state_msg->position.a2;
    qR_[2] = joi_state_msg->position.a3;
    qR_[3] = joi_state_msg->position.a4;
    qR_[4] = joi_state_msg->position.a5;
    qR_[5] = joi_state_msg->position.a6;
    qR_[6] = joi_state_msg->position.a7;
    publishFkine(qR_);
  }

  void joint_vel_cb(const iiwa_msgs::JointVelocity::ConstPtr &joi_state_msg) {
    TooN::Vector<7> qdotR;
    qdotR[0] = joi_state_msg->velocity.a1;
    qdotR[1] = joi_state_msg->velocity.a2;
    qdotR[2] = joi_state_msg->velocity.a3;
    qdotR[3] = joi_state_msg->velocity.a4;
    qdotR[4] = joi_state_msg->velocity.a5;
    qdotR[5] = joi_state_msg->velocity.a6;
    qdotR[6] = joi_state_msg->velocity.a7;
    publishVel(qR_, qdotR);
  }
};
} // namespace sun

#endif