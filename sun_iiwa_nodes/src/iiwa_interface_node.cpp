#include "ros/ros.h"
#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointVelocity.h"

iiwa_msgs::JointPosition msg_joi_pos;
ros::Publisher pubJointPosition;

void subJointPositionCB( const iiwa_msgs::JointPosition::ConstPtr& msg ){

    msg_joi_pos.position.a1 = msg->position.a1;
    msg_joi_pos.position.a2 = msg->position.a2;
    msg_joi_pos.position.a3 = msg->position.a3;
    msg_joi_pos.position.a4 = msg->position.a4;
    msg_joi_pos.position.a5 = msg->position.a5;
    msg_joi_pos.position.a6 = msg->position.a6;
    msg_joi_pos.position.a7 = msg->position.a7;

    msg_joi_pos.header.frame_id = msg->header.frame_id;
    msg_joi_pos.header.stamp = ros::Time::now();

    pubJointPosition.publish( msg_joi_pos );

}

iiwa_msgs::JointVelocity msg_joi_vel;
ros::Publisher pubJointVelocity;

void subJointVelocityCB( const iiwa_msgs::JointVelocity::ConstPtr& msg ){

    msg_joi_vel.velocity.a1 = msg->velocity.a1;
    msg_joi_vel.velocity.a2 = msg->velocity.a2;
    msg_joi_vel.velocity.a3 = msg->velocity.a3;
    msg_joi_vel.velocity.a4 = msg->velocity.a4;
    msg_joi_vel.velocity.a5 = msg->velocity.a5;
    msg_joi_vel.velocity.a6 = msg->velocity.a6;
    msg_joi_vel.velocity.a7 = msg->velocity.a7;

    msg_joi_vel.header.frame_id = msg->header.frame_id;
    msg_joi_vel.header.stamp = ros::Time::now();

    pubJointVelocity.publish( msg_joi_vel );

}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sun_iiwa_interface");

    ros::NodeHandle nh_public;
    
    ros::Subscriber subJointPosition = nh_public.subscribe("/iiwa/state/JointPosition", 1, subJointPositionCB);
    pubJointPosition = nh_public.advertise<iiwa_msgs::JointPosition>("/sun_iiwa/state/JointPosition", 1);

    ros::Subscriber subJointVelocity = nh_public.subscribe("/iiwa/state/JointVelocity", 1, subJointVelocityCB);
    pubJointVelocity = nh_public.advertise<iiwa_msgs::JointVelocity>("/sun_iiwa/state/JointVelocity", 1);

    ros::spin();
    
    return 0;
}