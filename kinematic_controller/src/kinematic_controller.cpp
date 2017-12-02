#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "dynamixel_msgs/JointState.h"
#include <math.h>
#include "dynamixel_controllers/SetSpeed.h"

int camX = 320;
int camY = 240;
float dataX = 0;
float dataY = 0;
float maxX = 89.5;
float maxY = 71.5;
int ball_detected = 0;
int motor_acceleration = 2000;
int stepperMaxSpeed = 300;

const float PI = 3.14159265359;
const float spoolD = 10;
const float motorPositionScale = 180/PI;

//starting lengths of cable
const float startingl1 = 57.27674048;
const float startingl2 = 57.27674048;
const float startingl3 = 57.27674048;
const float startingl4 = 57.27674048;

const int hyst = 2;
const int tolerance = 5;

bool boundFlag = false;
bool deadFlag = false;

struct lengthStruct{
	float l1;
	float l2;
	float l3;
	float l4;
	float nl1;
	float nl2;
	float nl3;
	float nl4;
	float x;
	float y;
};

struct speedStruct{
	float s1;
	float s2;
	float s3;
	float s4;
};




volatile bool goFlag = false;
float m1Position,m2Position,m3Position,m4Position;

void cameraDataCallback(const geometry_msgs::Point::ConstPtr& coordinate_msg){
	dataX = 89.5*(coordinate_msg->x-320)/640;
	dataY = 71.5*(coordinate_msg->y-240)/480;
	ball_detected = coordinate_msg->z;
	goFlag = true;

}

void m1Callback(const dynamixel_msgs::JointState::ConstPtr& m1State){
	m1Position = m1State->current_pos;

}

void m2Callback(const dynamixel_msgs::JointState::ConstPtr& m2State){
	m2Position = m2State->current_pos;
}

void m3Callback(const dynamixel_msgs::JointState::ConstPtr& m3State){
	m3Position = m3State->current_pos;
}

void m4Callback(const dynamixel_msgs::JointState::ConstPtr& m4State){
	m4Position = m4State->current_pos;
}

float motorPositionToLength(float motorPosition){
	float length;
	float rotations;
	rotations = (motorPositionScale * (motorPosition-PI)/360);
	rotations = (rotations)*4;
	length = rotations * PI * spoolD;
	return length;
}

float motorLengthToPosition(float length){
	float motorPosition;
	float rotations;
	rotations = length/(PI*spoolD);
	motorPosition = 360*(rotations)/motorPositionScale;
	motorPosition = (motorPosition/4);
	return motorPosition;
}

float checkAxisBounds(float cam, float current, float minimum, float maximum) {
	if ((current <= minimum) && (cam < 0)) {
		boundFlag = true;
		return (minimum - current - hyst);
	} else if ((current >= maximum) && (cam > 0)) {
		boundFlag = true;
		return (maximum - current + hyst);
	} else {
		boundFlag = false;
		return cam;
	}
}

float checkAxisDeadBand(float location, int iTolerance) {
	if ((location < tolerance) && (location > (tolerance * (-1)))) {
		deadFlag = true;
		return 0.0;
	} else {
		deadFlag = false;
		return location;
	}
}

struct camVals{
	float X;
	float Y;
};
camVals dataXYtoPub;
camVals check_boundaries(float camX, float camY, float currentX, float currentY) {
	camVals camXY;
	camX = checkAxisBounds(camX, currentX, (10.0), (maxX - 10.0));
	camY = checkAxisBounds(camY, currentY, (10.0), (maxY - 10.0));
	camXY.X = checkAxisDeadBand(camX, tolerance);
	camXY.Y = checkAxisDeadBand(camY, tolerance);
	return camXY;
}

lengthStruct findNewLengths(float dX, float dY){
	lengthStruct temp;
	camVals camXY;
	float m1Length = motorPositionToLength(m1Position);
	float m2Length = motorPositionToLength(m2Position);
	float m3Length = motorPositionToLength(m3Position);
	float m4Length = motorPositionToLength(m4Position);
	float currentl1 = startingl1 + m1Length;
	float currentl2 = startingl2 + m2Length;
	float currentl3 = startingl3 + m3Length;
	float currentl4 = startingl4 + m4Length;

	float currentX = maxX / 2 + (pow(currentl3,2.0) - pow(currentl4,2.0)) / (2 * maxX);
	float currentY = maxY / 2 + (pow(currentl4,2.0) - pow(currentl2,2.0)) / (2 * maxY);
	camXY = check_boundaries(dX,dY,currentX,currentY);
	dX = camXY.X;
	dY = camXY.Y;

	dataXYtoPub.X = dX;
	dataXYtoPub.Y = dY;

	float newL1 = sqrt(pow(      (currentX + dX),2.0)  + pow((maxY - (currentY + dY)),2.0));
	float newL2 = sqrt(pow((maxX - (currentX + dX)),2.0) + pow((maxY - (currentY + dY)),2.0));
	float newL3 = sqrt(pow(      (currentX + dX),2.0)  + pow(        (currentY + dY),2.0));
	float newL4 = sqrt(pow((maxX - (currentX + dX)),2.0) + pow(        (currentY + dY),2.0));
	temp.l1 = newL1 - currentl1;
	temp.l2 = newL2 - currentl2;
	temp.l3 = newL3 - currentl3;
	temp.l4 = newL4 - currentl4;
	temp.nl1 = newL1;
	temp.nl2 = newL2;
	temp.nl3 = newL3;
	temp.nl4 = newL4;
	temp.x = currentX;
	temp.y = currentY;
	return temp;
}


float maxSpeedRamp = 0.0;
float maxSpeed = 1.0;
float beta = 0.99;
float acceleration_scale(void) {
	if ((camX == 0) && (camY == 0)) {
		maxSpeedRamp = 0.0;
	} else {
		maxSpeedRamp = beta * maxSpeedRamp + (1.0 - beta) * maxSpeed;
		return maxSpeedRamp;  
	}
}

speedStruct preSpeeds;
speedStruct calculate_speeds(lengthStruct change_in_lengths) {
	speedStruct speeds;
	float diff1 = abs(change_in_lengths.l1);
	float diff2 = abs(change_in_lengths.l2);
	float diff3 = abs(change_in_lengths.l3);
	float diff4 = abs(change_in_lengths.l4);
	float speedScaler = std::max(diff1, diff2);
	speedScaler = std::max(speedScaler, diff3);
	speedScaler = std::max(speedScaler, diff4);
	if (speedScaler != 0.0){
		float speedMult1 = diff1 / speedScaler;
		float speedMult2 = diff2 / speedScaler;
		float speedMult3 = diff3 / speedScaler;
		float speedMult4 = diff4 / speedScaler;

		maxSpeedRamp = 4.0; //acceleration_scale();
		speeds.s1 = (maxSpeedRamp * speedMult1);
		speeds.s2 = (maxSpeedRamp * speedMult2);
		speeds.s3 = (maxSpeedRamp * speedMult3);
		speeds.s4 = (maxSpeedRamp * speedMult4);

	} else {
		speeds.s1 = 0.0;
		speeds.s2 = 0.0;
		speeds.s3 = 0.0;
		speeds.s4 = 0.0;
	}
	return speeds;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "kinematic_controller");
	ros::NodeHandle n;
	ros::ServiceClient client1 = n.serviceClient<dynamixel_controllers::SetSpeed>("/motor1_controller/set_speed");
	ros::ServiceClient client2 = n.serviceClient<dynamixel_controllers::SetSpeed>("/motor2_controller/set_speed");
	ros::ServiceClient client3 = n.serviceClient<dynamixel_controllers::SetSpeed>("/motor3_controller/set_speed");
	ros::ServiceClient client4 = n.serviceClient<dynamixel_controllers::SetSpeed>("/motor4_controller/set_speed");
	dynamixel_controllers::SetSpeed srv;
	std_msgs::Float64 m1NewPos;
	std_msgs::Float64 m2NewPos;
	std_msgs::Float64 m3NewPos;
	std_msgs::Float64 m4NewPos;
	geometry_msgs::Point XY;
	geometry_msgs::Point dataXYpub;
	geometry_msgs::Quaternion lengths;
	geometry_msgs::Quaternion positions;
	ros::Subscriber cameraSub = n.subscribe("coordinate_send_topic", 1, cameraDataCallback);
	ros::Subscriber motor1Sub = n.subscribe("/motor1_controller/state", 1, m1Callback);
	ros::Subscriber motor2Sub = n.subscribe("/motor2_controller/state", 1, m2Callback);
	ros::Subscriber motor3Sub = n.subscribe("/motor3_controller/state", 1, m3Callback);
	ros::Subscriber motor4Sub = n.subscribe("/motor4_controller/state", 1, m4Callback);
	ros::Publisher motor1Pub = n.advertise<std_msgs::Float64>("/motor1_controller/command", 1);
	ros::Publisher motor2Pub = n.advertise<std_msgs::Float64>("/motor2_controller/command", 1);
	ros::Publisher motor3Pub = n.advertise<std_msgs::Float64>("/motor3_controller/command", 1);
	ros::Publisher motor4Pub = n.advertise<std_msgs::Float64>("/motor4_controller/command", 1);
	ros::Publisher motorLengthPub = n.advertise<geometry_msgs::Quaternion>("motorLengths", 1);
	ros::Publisher motorPositionsPub = n.advertise<geometry_msgs::Quaternion>("motorPositions", 1);
	ros::Publisher PositionPub = n.advertise<geometry_msgs::Point>("current_pos", 1);
	ros::Publisher dataXYpublisher = n.advertise<geometry_msgs::Point>("dataXYpublisher", 1);
	
	bool flag1 = false;
	bool flag2 = false;
	bool flag3 = false;
	bool flag4 = false;


	ros::Rate loop_rate(50);
	lengthStruct newLengths;

	
	m1NewPos.data = PI;
	m2NewPos.data = PI;
	m3NewPos.data = PI;
	m4NewPos.data = PI;
	srv.request.speed = 5.0;
	client1.call(srv);
	client2.call(srv);
	client3.call(srv);
	client4.call(srv);
	ros::spinOnce();
	loop_rate.sleep();
	ros::spinOnce();
	loop_rate.sleep();
	motor1Pub.publish(m1NewPos);
	motor2Pub.publish(m2NewPos);
	motor3Pub.publish(m3NewPos);
	motor4Pub.publish(m4NewPos);
	ros::spinOnce();
	loop_rate.sleep();
	int microseconds = 0.5*1000*1000;
	usleep(microseconds);
	ros::spinOnce();
	loop_rate.sleep();
	srv.request.speed = 5.0;
	client1.call(srv);
	client2.call(srv);
	client3.call(srv);
	client4.call(srv);
	ros::spinOnce();
	loop_rate.sleep();
	ros::spinOnce();
	loop_rate.sleep();
	motor1Pub.publish(m1NewPos);
	motor2Pub.publish(m2NewPos);
	motor3Pub.publish(m3NewPos);
	motor4Pub.publish(m4NewPos);
	ros::spinOnce();
	microseconds = 3*1000*1000;
	usleep(microseconds);
	ros::spinOnce();
	loop_rate.sleep();
	ros::spinOnce();
	loop_rate.sleep();

	while (ros::ok()){
		if (goFlag = true){

			newLengths = findNewLengths(dataX, dataY);
			speedStruct motorSpeeds = calculate_speeds(newLengths);
			XY.x = newLengths.x;
			XY.y = newLengths.y;
			lengths.x = newLengths.nl1;
			lengths.y = newLengths.nl2;
			lengths.z = newLengths.nl3;
			lengths.w = newLengths.nl4;
			positions.x = m1Position;
			positions.y = m2Position;
			positions.z = m3Position;
			positions.w = m4Position;
			PositionPub.publish(XY);
			motorLengthPub.publish(lengths);
			motorPositionsPub.publish(positions);

			dataXYpub.x = dataXYtoPub.X;
			dataXYpub.y = dataXYtoPub.Y;
			dataXYpublisher.publish(dataXYpub);


			srv.request.speed = motorSpeeds.s1;
			if (client1.call(srv)){
				flag1 = true;
			} else {
				ROS_ERROR("Failed to call service 1");
				flag1 = false;
			}

			srv.request.speed = motorSpeeds.s2;
			if (client2.call(srv)){
				flag2 = true;
			} else {
				ROS_ERROR("Failed to call service 2");
				flag2 = false;
			}

			srv.request.speed = motorSpeeds.s3;
			if (client3.call(srv)){
				flag3 = true;
			} else {
				ROS_ERROR("Failed to call service 3");
				flag3 = false;
			}

			srv.request.speed = motorSpeeds.s4;
			if (client4.call(srv)){
				flag4 = true;

			} else {
				ROS_ERROR("Failed to call service 4");
				flag4 = false;
			}
			if (flag1 && flag2 && flag3 && flag4){
				ROS_INFO("Successfully published all speeds. \n Speed1 =  %f Speed2 =  %f Speed3 =  %f Speed4 =  %f \n", motorSpeeds.s1, motorSpeeds.s2, motorSpeeds.s3, motorSpeeds.s4);			

			}

			m1NewPos.data = m1Position + motorLengthToPosition(newLengths.l1);
			m2NewPos.data = m2Position + motorLengthToPosition(newLengths.l2);
			m3NewPos.data = m3Position + motorLengthToPosition(newLengths.l3);
			m4NewPos.data = m4Position + motorLengthToPosition(newLengths.l4);

			motor1Pub.publish(m1NewPos);
			motor2Pub.publish(m2NewPos);
			motor3Pub.publish(m3NewPos);
			motor4Pub.publish(m4NewPos);
		}
		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}
