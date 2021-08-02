#ifndef SUN_IIWA_NODES_H_
#define SUN_IIWA_NODES_H_

#include "iiwa_msgs/JointPosition.h"
#include "sun_robot_lib/Robots/LBRiiwa7.h"
#include "sun_robot_ros/FkineNode.h"

namespace sun {

class IIWA_FkineNode : public FkineNode {
private:
  /* data */
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
  }

  void
  joint_position_cb(const iiwa_msgs::JointPosition::ConstPtr &joi_state_msg) {
    TooN::Vector<7> qR;
    qR[0] = joi_state_msg->position.a1;
    qR[1] = joi_state_msg->position.a2;
    qR[2] = joi_state_msg->position.a3;
    qR[3] = joi_state_msg->position.a4;
    qR[4] = joi_state_msg->position.a5;
    qR[5] = joi_state_msg->position.a6;
    qR[6] = joi_state_msg->position.a7;
    publishFkine(qR);
  }
};
} // namespace sun

#endif