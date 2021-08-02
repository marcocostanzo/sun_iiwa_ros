#include "iiwa_msgs/JointPosition.h"
#include "ros/ros.h"
#include "sun_robot_lib/Robots/LBRiiwa7.h"
#include "sun_robot_ros/ClikNode.h"

namespace sun {
class iiwa_ClikNode : public ClikNode {
private:
protected:
  ros::Publisher pub_joints_cmd_;
  ros::Subscriber joint_position_sub_;

public:
  iiwa_ClikNode(const ros::NodeHandle &nh_for_topics = ros::NodeHandle("clik"),
                const ros::NodeHandle &nh_for_parmas = ros::NodeHandle("~"))
      : ClikNode(std::make_shared<LBRiiwa7>("iiwa7"), nh_for_topics,
                 nh_for_parmas) {
                   
    std::string joint_state_topic_str;
    nh_for_parmas.param("joint_state_topic", joint_state_topic_str,
                        std::string("/iiwa/state/JointPosition"));
    std::string joint_command_topic_str;
    nh_for_parmas.param("joint_command_topic", joint_command_topic_str,
                        std::string("/iiwa/command/JointPosition"));

    // Subscribers
    joint_position_sub_ = nh_.subscribe(
        joint_state_topic_str, 1, &iiwa_ClikNode::joint_position_cb, this);

    // Publishers
    pub_joints_cmd_ =
        nh_.advertise<iiwa_msgs::JointPosition>(joint_command_topic_str, 1);
  }

  TooN::Vector<7> qR;
  bool b_joint_state_arrived = false;
  void
  joint_position_cb(const iiwa_msgs::JointPosition::ConstPtr &joi_state_msg) {

    qR[0] = joi_state_msg->position.a1;
    qR[1] = joi_state_msg->position.a2;
    qR[2] = joi_state_msg->position.a3;
    qR[3] = joi_state_msg->position.a4;
    qR[4] = joi_state_msg->position.a5;
    qR[5] = joi_state_msg->position.a6;
    qR[6] = joi_state_msg->position.a7;
    b_joint_state_arrived = true;
  }

  //! Cbs
  virtual TooN::Vector<> getJointPositionRobot(bool wait_new_sample) override {
    // wait joint position
    if (wait_new_sample) {
      b_joint_state_arrived = false;
    }
    while (ros::ok() && !b_joint_state_arrived) {
      spinOnce();
    }
    return qR;
  }
  virtual void publishJointRobot(const TooN::Vector<> &qR,
                                 const TooN::Vector<> &qR_dot) override {

    iiwa_msgs::JointPosition out_msg;

    out_msg.position.a1 = qR[0];
    out_msg.position.a2 = qR[1];
    out_msg.position.a3 = qR[2];
    out_msg.position.a4 = qR[3];
    out_msg.position.a5 = qR[4];
    out_msg.position.a6 = qR[5];
    out_msg.position.a7 = qR[6];
    /*
        out_msg.velocity.a1 = dqR[0];
        out_msg.velocity.a2 = dqR[1];
        out_msg.velocity.a3 = dqR[2];
        out_msg.velocity.a4 = dqR[3];
        out_msg.velocity.a5 = dqR[4];
        out_msg.velocity.a6 = dqR[5];
        out_msg.velocity.a7 = dqR[6];
    */
    out_msg.header.frame_id = "iiwa";
    out_msg.header.stamp = ros::Time::now();

    pub_joints_cmd_.publish(out_msg);
  }
};
} // namespace sun

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "iiwa_clik");

  ros::NodeHandle nh_private = ros::NodeHandle("~");
  ros::NodeHandle nh_public;

  sun::iiwa_ClikNode clik_node(nh_public, nh_private);

  clik_node.run();
}