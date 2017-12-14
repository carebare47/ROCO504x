#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include <stdlib.h>
bool throwControl = false;
const float clampClose = 5.0;
//clamp1 = 5
//clamp2 = 5.2
//clamp3 = 5.2
//clamp4 = 5.2
//



const float clampOpen = 2.5;
bool step = false;
void FrameClamps_cb(const std_msgs::Bool::ConstPtr& clamps_command){
	throwControl = clamps_command->data;
}
/*
void step_pub_cb(const std_msgs::Bool::ConstPtr& booleybooley){
	step = booleybooley->data;
}
*/
void init_kinematic_controller(ros::Publisher);

int main(int argc, char **argv){

	ros::init(argc, argv, "throw_controller");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	ros::Publisher Clamp1Pub = n.advertise<std_msgs::Float64>("/frameclamp1_controller/command", 1);
	ros::Publisher Clamp2Pub = n.advertise<std_msgs::Float64>("/frameclamp2_controller/command", 1);
	ros::Publisher Clamp3Pub = n.advertise<std_msgs::Float64>("/frameclamp3_controller/command", 1);
	ros::Publisher Clamp4Pub = n.advertise<std_msgs::Float64>("/frameclamp4_controller/command", 1);
	ros::Publisher Motor3Pub = n.advertise<std_msgs::Float64>("/motor3_controller/command", 1);
	ros::Publisher Motor4Pub = n.advertise<std_msgs::Float64>("/motor4_controller/command", 1);
	ros::Publisher finished_pub = n.advertise<std_msgs::Bool>("/finished_throwing", 1);

	ros::Publisher ThrowClampPub = n.advertise<std_msgs::Float64>("/throwclamp_controller/command", 1);
	ros::Publisher ThrowSpoolPub = n.advertise<std_msgs::Int16>("/throw_motor", 1);
	ros::Publisher init_kinematic_controller_pub = n.advertise<std_msgs::Bool>("/init_frame", 1);

	ros::Publisher gripperPub = n.advertise<std_msgs::Float64>("/gripper_controller/command", 1);

	std_msgs::Float64 length_change;
	ros::Publisher throw_length_change = n.advertise<std_msgs::Float64>("/throw_length_change", 1);

	ros::Subscriber FrameClampsSub = n.subscribe("/frame_clamps_command", 1, FrameClamps_cb);
  	//ros::Subscriber step_pub = n.subscribe("/throw_steps", 1, step_pub_cb);
	std_msgs::Float64 gripper;
	std_msgs::Bool finished;
	std_msgs::Float64 cFrameClamps;
	std_msgs::Float64 cThrowClamp;
	std_msgs::Int16 cThrowSpool;
	std_msgs::Float64 m3Command;
	std_msgs::Float64 m4Command;
	int Wind = 1;
	int unWind = 2;
	int spoolTimeWind = 5;
	int spoolTimeUnwind = 5;
	while (ros::ok()){
		if (throwControl){

			init_kinematic_controller(init_kinematic_controller_pub);

			gripper.data = 3.1;
			gripperPub.publish(gripper);

    		//close frame clamps
			cFrameClamps.data = clampClose;
    		//start winder winding
			cThrowSpool.data = Wind;
    		//close throw clamp
			cThrowClamp.data = 1.8;

    		//publish frame clamps closed
			Clamp1Pub.publish(cFrameClamps);
			Clamp2Pub.publish(cFrameClamps);
				//Clamp3Pub.publish(cFrameClamps);
    		//cFrameClamps.data = 0;
				//Clamp4Pub.publish(cFrameClamps);

			length_change.data = -31.0;
			throw_length_change.publish(length_change);
			sleep(2);

			Clamp3Pub.publish(cFrameClamps);
			Clamp4Pub.publish(cFrameClamps);


			sleep(2);

    		////begin winding
			ThrowSpoolPub.publish(cThrowSpool);
    		//wait until wound
			sleep(11);

    		//sleep(spoolTimeWind);

    		//stop winding
			cThrowSpool.data = 0;
			ThrowSpoolPub.publish(cThrowSpool);
    		//shut throw clamp
			ThrowClampPub.publish(cThrowClamp);
			sleep(2);
				//tension bottom elastic
				// m3Command.data = something;
				// m4Command.data = something;
				// Motor3Pub.publish(m3Command);
				// Motor4Pub.publish(m4Command);







    		//begin unwinding
			cThrowSpool.data = unWind;
			ThrowSpoolPub.publish(cThrowSpool);
			sleep(11);
    		//stop unwinding
			cThrowSpool.data = 0;
			ThrowSpoolPub.publish(cThrowSpool);

    		//open throw clamp (throw!)
			gripper.data = 3.4;
			gripperPub.publish(gripper);
			cThrowClamp.data = 4.0;
			ThrowClampPub.publish(cThrowClamp);
			sleep(4);

    		//open frame clamps
			cFrameClamps.data = clampOpen;
			Clamp1Pub.publish(cFrameClamps);
			Clamp2Pub.publish(cFrameClamps);
			Clamp3Pub.publish(cFrameClamps);
			Clamp4Pub.publish(cFrameClamps);

			sleep(1);

			init_kinematic_controller(init_kinematic_controller_pub);
			finished.data = true;
			finished_pub.publish(finished);
			throwControl = false;

		}
		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}

void init_kinematic_controller(ros::Publisher init_kinematic_controller_pub){
	std_msgs::Bool initialise_kinematic_controller;
	initialise_kinematic_controller.data = true;
	init_kinematic_controller_pub.publish(initialise_kinematic_controller);
	sleep(3);
	initialise_kinematic_controller.data = false;
	init_kinematic_controller_pub.publish(initialise_kinematic_controller);
}