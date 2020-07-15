#include "ros/ros.h"
#include "TooN/TooN.h"
#include "sun_robot_lib/Robots/LBRiiwa7.h"
#include "sensor_msgs/JointState.h"
#include <std_msgs/Float64MultiArray.h> 
#include "sun_traj_lib/Vector_Independent_Traj.h"
#include "sun_traj_lib/Quintic_Poly_Traj.h"
#include <iiwa_msgs/JointPosition.h>

using namespace std;
using namespace TooN;

#define FRAME_RATE 1000

Vector<7> q0, qf;
           								
double tf = 25.0;
bool joint_ok_init = false;
iiwa_msgs::JointPosition current_joint_position;
	
void readJointPos(const iiwa_msgs::JointPosition& msg)
{
      if(!joint_ok_init){
		   joint_ok_init = true;
		   current_joint_position = msg;
		   q0[0] = current_joint_position.position.a1;
		   q0[1] = current_joint_position.position.a2;
		   q0[2] = current_joint_position.position.a3;
		   q0[3] = current_joint_position.position.a4;
		   q0[4] = current_joint_position.position.a5;
		   q0[5] = current_joint_position.position.a6;
		   q0[6] = current_joint_position.position.a7; 
   	}
}

int main(int argc, char*argv[]){

	ros::init(argc, argv, "iiwa_move_to");
   ros::NodeHandle nh;
    
   //Oggetto robot
	sun::LBRiiwa7 iiwa;
	
	qf = makeVector(atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]), atof(argv[5]), atof(argv[6]), atof(argv[7]));
	 
   ros::Subscriber joint_states_sub = nh.subscribe("/iiwa/state/JointPosition", 1, readJointPos);
	ros::Publisher joint_states_pub = nh.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1);
	
	while(ros::ok() && !joint_ok_init){
		ros::spinOnce();
	}
	
	//GENERAZIONE TRAIETTORIA
	sun::Vector_Independent_Traj traj;	
	
	for(int i=0;i < iiwa.getNumJoints(); i++){
		traj.push_back_traj(sun::Quintic_Poly_Traj (tf, q0[i], qf[i]));
	}
	
   ros::Rate loop_rate(FRAME_RATE);

   double time_now = ros::Time::now().toSec();
    
   traj.changeInitialTime(time_now);
    
   std_msgs::Float64MultiArray joints_states_msg;
   joints_states_msg.data.resize(iiwa.getNumJoints()*2);
    
   while(ros::ok() && (!traj.isCompleate(time_now))){
    
		 time_now = ros::Time::now().toSec();
		 
		 Vector<> position = traj.getPosition(time_now);
		 Vector<> velocity = traj.getVelocity(time_now);
		 
		 vector<bool> check_joint_lim = iiwa.checkHardJointLimits(position);
		 vector<bool> check_vel_lim = iiwa.checkHardVelocityLimits(velocity);

		 for(int i = 0; i < iiwa.getNumJoints(); i++){
		 	if(check_joint_lim[i]){
		   	cout<<"Il giunto " << i+1 << " ha superato il limite, valore " << position[i] << endl;
		   	return -1;
		   }
		   if(check_vel_lim[i]){
		   	cout<<"Il giunto " << i+1 << " ha superato il limite di velocità, valore " << velocity[i] << endl;
		      return -1;
		   }
		 }
		 
		 iiwa_msgs::JointPosition outmsg;
		 outmsg.position.a1 = position[0]; 
		 outmsg.position.a2 = position[1];
		 outmsg.position.a3 = position[2];
		 outmsg.position.a4 = position[3];
		 outmsg.position.a5 = position[4];
		 outmsg.position.a6 = position[5];
		 outmsg.position.a7 = position[6];

		joint_states_pub.publish(outmsg);
		loop_rate.sleep();
	}

}
