#include "ros/ros.h"
#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointVelocity.h"

#define NUM_J 7

bool b_initialized = false;

std::vector<double> joi_comm;
void subJointCommandCB( const iiwa_msgs::JointPosition::ConstPtr& msg ){

    if( !b_initialized ){
        return;
    }

    joi_comm[0] = msg->position.a1;
    joi_comm[1] = msg->position.a2;
    joi_comm[2] = msg->position.a3;
    joi_comm[3] = msg->position.a4;
    joi_comm[4] = msg->position.a5;
    joi_comm[5] = msg->position.a6;
    joi_comm[6] = msg->position.a7;

}

std::vector<double> gains;
iiwa_msgs::JointVelocity vel_command;
ros::Publisher pubJointVelocity;
void subJointPositionCB( const iiwa_msgs::JointPosition::ConstPtr& msg ){

    if( !b_initialized ){
        joi_comm[0] = msg->position.a1;
        joi_comm[1] = msg->position.a2;
        joi_comm[2] = msg->position.a3;
        joi_comm[3] = msg->position.a4;
        joi_comm[4] = msg->position.a5;
        joi_comm[5] = msg->position.a6;
        joi_comm[6] = msg->position.a7;
        b_initialized = true;
        ROS_INFO("Initialized!");
        return;
    }

    vel_command.velocity.a1 = gains[0] * ( joi_comm[0] - msg->position.a1);
    vel_command.velocity.a2 = gains[1] * ( joi_comm[1] - msg->position.a2);
    vel_command.velocity.a3 = gains[2] * ( joi_comm[2] - msg->position.a3);
    vel_command.velocity.a4 = gains[3] * ( joi_comm[3] - msg->position.a4);
    vel_command.velocity.a5 = gains[4] * ( joi_comm[4] - msg->position.a5);
    vel_command.velocity.a6 = gains[5] * ( joi_comm[5] - msg->position.a6);
    vel_command.velocity.a7 = gains[6] * ( joi_comm[6] - msg->position.a7);

    vel_command.header.stamp = ros::Time::now();

    pubJointVelocity.publish(vel_command);

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "iiwa_joint_controller");
    
    ros::NodeHandle nh_public;
    
    ros::Subscriber subJointPosition = nh_public.subscribe("/iiwa/state/JointPosition", 1, subJointPositionCB);

    ros::Subscriber subJointCommand = nh_public.subscribe("/sun_iiwa/command/JointPosition", 1, subJointCommandCB);
    
    pubJointVelocity = nh_public.advertise<iiwa_msgs::JointVelocity>("/iiwa/command/JointVelocity", 1);

    joi_comm.resize(NUM_J);

    gains.resize(NUM_J);
    for(int i = 0; i<NUM_J; i++){
        gains[i] = 1.0;
    }

    ros::spin();    

    return 0;
}