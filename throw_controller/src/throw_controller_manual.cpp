#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include <stdlib.h>
int throwControl = -1;
const float clampClose = 4.5;
const float clampOpen = 2.5;
bool step = false;
void FrameClamps_cb(const std_msgs::Int16::ConstPtr& clamps_command){
  throwControl = clamps_command->data;
}
void step_pub_cb(const std_msgs::Bool::ConstPtr& booleybooley){
	step = booleybooley->data;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "throw_controller");
  	ros::NodeHandle n;
  	ros::Rate loop_rate(100);

  	ros::Publisher Clamp1Pub = n.advertise<std_msgs::Float64>("/frameclamp1_controller/command", 1);
  	ros::Publisher Clamp2Pub = n.advertise<std_msgs::Float64>("/frameclamp2_controller/command", 1);
  	ros::Publisher Clamp3Pub = n.advertise<std_msgs::Float64>("/frameclamp3_controller/command", 1);
  	ros::Publisher Clamp4Pub = n.advertise<std_msgs::Float64>("/frameclamp4_controller/command", 1);
	ros::Publisher ThrowClampPub = n.advertise<std_msgs::Float64>("/throwclamp_controller/command", 1);
	ros::Publisher ThrowSpoolPub = n.advertise<std_msgs::Int16>("/throw_motor", 1);

  	ros::Subscriber FrameClampsSub = n.subscribe("/frame_clamps_command", 1, FrameClamps_cb);
  	ros::Subscriber step_pub = n.subscribe("/throw_steps", 1, step_pub_cb);

  	std_msgs::Float64 cFrameClamps;
  	std_msgs::Float64 cThrowClamp;
  	std_msgs::Int16 cThrowSpool;
  	int Wind = 1;
  	int unWind = 2;
  	int spoolTimeWind = 5;
  	int spoolTimeUnwind = 5;
  	while (ros::ok()){
    	if (throwControl == 1){

    		//close frame clamps
    		cFrameClamps.data = clampClose;
    		//start winder winding
    		cThrowSpool.data = Wind;
    		//close throw clamp
    		cThrowClamp.data = 2;

    		//publish frame clamps closed
    		Clamp1Pub.publish(cFrameClamps);
    		Clamp2Pub.publish(cFrameClamps);
    		Clamp3Pub.publish(cFrameClamps);
    		//cFrameClamps.data = 0;
    		Clamp4Pub.publish(cFrameClamps);
    		sleep(2);

    		////begin winding
    		ThrowSpoolPub.publish(cThrowSpool);
    		//wait until wound
    		while (!step){

    		}
    		step = false;
    		//sleep(spoolTimeWind);

    		//stop winding
    		cThrowSpool.data = 0;
    		ThrowSpoolPub.publish(cThrowSpool);
    		//shut throw clamp
    		ThrowClampPub.publish(cThrowClamp);
    		sleep(2);

    		//begin unwinding
    		cThrowSpool.data = unWind;
    		ThrowSpoolPub.publish(cThrowSpool);
    		sleep(spoolTimeUnwind);
    		//stop unwinding
    		cThrowSpool.data = 0;
    		ThrowSpoolPub.publish(cThrowSpool);
    		
    		//open throw clamp (throw!)
    		cThrowClamp.data = 4;
    		ThrowClampPub.publish(cThrowClamp);
    		sleep(5);

    		//open frame clamps
    		cFrameClamps.data = clampOpen;
    		Clamp1Pub.publish(cFrameClamps);
    		Clamp2Pub.publish(cFrameClamps);
    		Clamp3Pub.publish(cFrameClamps);
    		Clamp4Pub.publish(cFrameClamps);
    		throwControl = 0;

    	}
    	ros::spinOnce();
    	loop_rate.sleep();

  	}
	return 0;
}