#include "iiwa_msgs/JointPosition.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include "sun_robot_ros/utils.h"

std::vector<std::string> jont_names = { "iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4",
                                        "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7" };

ros::Publisher pubJointCommand;

void subJointStateCommandCB(sensor_msgs::JointState msg)
{
  msg = sun::filterJointNames(msg, jont_names);
  iiwa_msgs::JointPosition cmd_msg;
  cmd_msg.header = msg.header;
  cmd_msg.position.a1 = msg.position[0];
  cmd_msg.position.a2 = msg.position[1];
  cmd_msg.position.a3 = msg.position[2];
  cmd_msg.position.a4 = msg.position[3];
  cmd_msg.position.a5 = msg.position[4];
  cmd_msg.position.a6 = msg.position[5];
  cmd_msg.position.a7 = msg.position[6];

  pubJointCommand.publish(cmd_msg);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "iiwa_joint_state_command_interface");

  ros::NodeHandle nh_public;

  ros::Subscriber subJointStateCommand =
      nh_public.subscribe("/iiwa/command/joint_state_position", 1, subJointStateCommandCB);
  pubJointCommand = nh_public.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1);

  ros::spin();

  return 0;
}