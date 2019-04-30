#include "ros/ros.h"
#include "Robots/LBRiiwa7.h"
#include "sun_robot_ros/CLIK_Node.h"
#include "iiwa_msgs/JointPosition.h"
#include "geometry_msgs/PoseStamped.h"

using namespace std;
using namespace TooN;

#ifndef SUN_COLORS
#define SUN_COLORS

/* ======= COLORS ========= */
#define CRESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLD    "\033[1m"       /* Bold */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */
/*===============================*/

#endif

#define HEADER_PRINT BOLDYELLOW "[" << ros::this_node::getName() << "] " CRESET 


/*GLOBAL ROS VARS*/
ros::Publisher pub_joints;
ros::Publisher pub_pose;
/*END Global ROS Vars*/

CLIK_Node* clik_node;

Vector<7> qR;
bool b_joint_state_arrived = false;
void readJointState( const iiwa_msgs::JointPosition::ConstPtr& joi_state_msg ){

    qR[0] = joi_state_msg->position.a1;
    qR[1] = joi_state_msg->position.a2;
    qR[2] = joi_state_msg->position.a3;
    qR[3] = joi_state_msg->position.a4;
    qR[4] = joi_state_msg->position.a5;
    qR[5] = joi_state_msg->position.a6;
    qR[6] = joi_state_msg->position.a7;
    b_joint_state_arrived = true;

    Matrix<4,4> b_T_e = clik_node->getRobot()->fkine( clik_node->getRobot()->joints_Robot2DH(qR) );

    UnitQuaternion uq(b_T_e);
    Vector<3> p = transl(b_T_e);

    geometry_msgs::PoseStamped out_msg;

    out_msg.header = joi_state_msg->header;
    out_msg.pose.position.x = p[0];
    out_msg.pose.position.y = p[1];
    out_msg.pose.position.z = p[2];
    out_msg.pose.orientation.w = uq.getS();
    Vector<3> uq_v = uq.getV();
    out_msg.pose.orientation.x = uq_v[0];
    out_msg.pose.orientation.y = uq_v[1];
    out_msg.pose.orientation.z = uq_v[2];

    pub_pose.publish(out_msg);
    
}

Vector<> getJointPosition_fcn(){

    //wait joint position
    
    //b_joint_state_arrived = false;
    while( ros::ok() && !b_joint_state_arrived )
    {
        ros::spinOnce();
    }
    

    return qR;

}

void publish_fcn( const Vector<>& qR, const Vector<>& dqR ){
    iiwa_msgs::JointPosition out_msg;

    out_msg.position.a1 = qR[0];
    out_msg.position.a2 = qR[1];
    out_msg.position.a3 = qR[2];
    out_msg.position.a4 = qR[3];
    out_msg.position.a5 = qR[4];
    out_msg.position.a6 = qR[5];
    out_msg.position.a7 = qR[6];

    out_msg.header.frame_id = "SIA5F";
    out_msg.header.stamp = ros::Time::now();

    pub_joints.publish(out_msg);
}

int main(int argc, char *argv[])
{

    ros::init(argc,argv, "iiwa_clik");

    ros::NodeHandle nh_private = ros::NodeHandle("~");
    ros::NodeHandle nh_public = ros::NodeHandle();

    //params
    string topic_joint_command_str;
    nh_private.param("joint_command_topic" , topic_joint_command_str, string("/iiwa/command/JointPosition") );
    string topic_ee_pose_str;
    nh_private.param("ee_pose_topic" , topic_ee_pose_str, string("ee_pose") );
    string joint_state_topic_str;
    nh_private.param("joint_state_topic" , joint_state_topic_str, string("/iiwa/state/JointPosition") );

    //Subscribers
    ros::Subscriber joint_position_sub = nh_public.subscribe(joint_state_topic_str, 1, readJointState);
    
    //Publishers
    pub_joints = nh_public.advertise<iiwa_msgs::JointPosition>(topic_joint_command_str, 1);
    pub_pose = nh_public.advertise<geometry_msgs::PoseStamped>(topic_ee_pose_str, 1);
    
    clik_node = new CLIK_Node(
            LBRiiwa7( "iiwa7" ),
            nh_public,
            nh_private,
            getJointPosition_fcn, 
            publish_fcn
            );

    clik_node->run();

    delete clik_node;

    return 0;
}