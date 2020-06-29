#include "ros/ros.h"
#include "sun_robot_lib/Robots/LBRiiwa7.h"
#include "sun_robot_ros/clikNode.h"
#include "iiwa_msgs/JointPosition.h"


/*GLOBAL ROS VARS*/
ros::Publisher pub_joints_cmd;
std::string joint_state_topic_str;
/*END Global ROS Vars*/

TooN::Vector<7> qR;
bool b_joint_state_arrived = false;
void joint_position_cb( const iiwa_msgs::JointPosition::ConstPtr& joi_state_msg ){

    qR[0] = joi_state_msg->position.a1;
    qR[1] = joi_state_msg->position.a2;
    qR[2] = joi_state_msg->position.a3;
    qR[3] = joi_state_msg->position.a4;
    qR[4] = joi_state_msg->position.a5;
    qR[5] = joi_state_msg->position.a6;
    qR[6] = joi_state_msg->position.a7;
    b_joint_state_arrived = true;
    
}

TooN::Vector<> get_joint_position_fcn()
{
    //wait joint position
    ros::NodeHandle nh_public;
    ros::Subscriber joint_position_sub = nh_public.subscribe(joint_state_topic_str, 1, joint_position_cb);
    b_joint_state_arrived = false;
    while( ros::ok() && !b_joint_state_arrived )
    {
        ros::spinOnce();
    }
    return qR;
}



void joint_publish_fcn( const TooN::Vector<>& qR, const TooN::Vector<>& dqR ){

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

    pub_joints_cmd.publish(out_msg);
}

int main(int argc, char *argv[])
{

    ros::init(argc,argv, "iiwa_clik");

    ros::NodeHandle nh_private = ros::NodeHandle("~");
    ros::NodeHandle nh_public;

    //params
    nh_private.param("joint_state_topic" , joint_state_topic_str, std::string("/iiwa/state/JointPosition") );
    std::string joint_command_topic_str;
    nh_private.param("joint_command_topic" , joint_command_topic_str, std::string("/iiwa/command/JointPosition") );

    //Subscribers

    //Publishers
    pub_joints_cmd = nh_public.advertise<iiwa_msgs::JointPosition>(joint_command_topic_str, 1);

    sun::LBRiiwa7 iiwa7("iiwa7");
    sun::clikNode clik_node(
            iiwa7, 
            nh_public,
            nh_private,
            get_joint_position_fcn, 
            joint_publish_fcn
            );

    clik_node.run();

}