#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "dynamixel_msgs/JointState.h"
#include "ros/time.h"
#include <math.h>
#include "dynamixel_controllers/SetSpeed.h"
#include "boost/thread.hpp"
#include "boost/chrono.hpp"
#include "geometry_msgs/PointStamped.h"

float globalSpeedScalar = 20.0;
int camX = 320;
int camY = 240;
float dataX = 0;
float dataY = 0;
float maxX = 89.5;
float maxY = 71.5;
int ball_detected = 0;
int motor_acceleration = 2000;
int stepperMaxSpeed = 300;
int tension = 0;
float gripper_side = 10.0;
float gripper_diagonal = 14.142;
float half_gripper_side = gripper_side/2;
float half_gripper_diagonal = gripper_diagonal/2;
float maxSpeedRamp = 0.0;
float maxSpeed = 10.0;
float beta = 0.99;
float throw_length_change = 0;
bool throwFlag = false;

const float PI = 3.14159265359;
const float spoolD = 10.0;
const float motorPositionScale = 180/PI;

//starting lengths of cable
const float startingl1 = 57.27674048 - half_gripper_diagonal;
const float startingl2 = 57.27674048 - half_gripper_diagonal;
const float startingl3 = 57.27674048 - half_gripper_diagonal;
const float startingl4 = 57.27674048 - half_gripper_diagonal;
const int hyst = 0;
const int tolerance = 0;

bool boundFlag = false;
bool deadFlag = false;
bool goFlag = false;
bool initFlag = true;
float m1Position,m2Position,m3Position,m4Position;

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

struct camVals{
	float X;
	float Y;
};

struct threadStruct{
	ros::ServiceClient client;
	dynamixel_controllers::SetSpeed srv;
	float speed;
};

dynamixel_controllers::SetSpeed srv;
std_msgs::Float64 m1NewPos;
std_msgs::Float64 m2NewPos;
std_msgs::Float64 m3NewPos;
std_msgs::Float64 m4NewPos;
std_msgs::Bool bound_flag_obj;
dynamixel_msgs::JointState motor1sim;
dynamixel_msgs::JointState motor2sim;
dynamixel_msgs::JointState motor3sim;
dynamixel_msgs::JointState motor4sim;
geometry_msgs::Point XY;
geometry_msgs::Quaternion lengths;
geometry_msgs::Quaternion positions;
lengthStruct newLengths;

void proportionalControl(int X, int Y){
	int proportional = pow(X, 2.0) + pow(Y, 2.0);
	globalSpeedScalar = 0.3944444	 * proportional;
//return globalSpeedScalar;
}

void cameraDataCallback(const geometry_msgs::Point::ConstPtr& coordinate_msg){
	dataX = 89.5*(coordinate_msg->x-320)/640.0;
	dataY = 71.5*(coordinate_msg->y-240)/480.0;
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

void initCB(const std_msgs::Bool::ConstPtr& init){

	//if (init->data == 1)
		initFlag = init->data;//true;
}

void throw_length_changeCB(const std_msgs::Float64::ConstPtr& change){
throw_length_change = change->data;
throwFlag = true;
}




float motorPositionToLength2(float motorPosition){
	float length;
	float rotations;
	rotations = (motorPositionScale * (motorPosition)/360);
	rotations = (rotations)*4.0;
	length = rotations * PI * spoolD;
	return length;
}

float motorPositionToLength(float motorPosition){
	float length;
	float rotations;
	rotations = (motorPositionScale * (motorPosition-PI)/360);
	rotations = (rotations)*4.0;
	length = rotations * PI * spoolD;
	return length;
}

float motorLengthToPosition(float length){
	float motorPosition;
	float rotations;
	rotations = (length)/(PI*spoolD);
	motorPosition = 360*(rotations)/motorPositionScale;
	motorPosition = (motorPosition/4.0);
	return motorPosition;
}

float checkAxisBounds(float cam, float current, float minimum, float maximum) {
	if (((current <= minimum) && (cam < 0))||(cam == 0.0)) {
		boundFlag = true;
		return (0.0);
	} else if (((current >= maximum) && (cam > 0))||(cam == 0.0)) {
		boundFlag = true;
		return (0.0);
	} else {
		boundFlag = false;
		return cam;
	}
}

float checkAxisDeadBand(float location, int iTolerance) {
	if ((location < tolerance) && (location > (tolerance * (-1)))) {
		deadFlag = true;
		return 0;
	} else {
		deadFlag = false;
		return location;
	}
}

camVals check_boundaries(float camX, float camY, float currentX, float currentY) {
	camVals camXY;
	camX = checkAxisBounds(camX, currentX, (10), (maxX - 20));
	camY = checkAxisBounds(camY, currentY, (10), (maxY - 20));
	camXY.X = checkAxisDeadBand(camX, 89.5*tolerance/640);
	camXY.Y = checkAxisDeadBand(camY, 71.5*tolerance/480);
	return camXY;
}

lengthStruct findNewLengths(float dX, float dY){
	lengthStruct temp;
	camVals camXY;
	float m1Length = motorPositionToLength(m1Position+0.35355+tension);
	float m2Length = motorPositionToLength(m2Position+0.35355+tension);
	float m3Length = motorPositionToLength(m3Position+0.35355+tension);
	float m4Length = motorPositionToLength(m4Position+0.35355+tension);
	float currentl1 = m1Length + startingl1;
	float currentl2 = m2Length + startingl2;
	float currentl3 = m3Length + startingl3;
	float currentl4 = m4Length + startingl4;
	ROS_INFO("l1 = %f,l2 = %f,l3 = %f, l4 = %f",currentl1,currentl2,currentl3,currentl4);
	float currentX = (maxX-gripper_side)/2 + (pow(currentl3,2.0) - pow(currentl4,2.0))/(2*(maxX-gripper_side));
	float currentY = sqrt(pow(currentl3,2.0) - pow(currentX,2.0));
	ROS_INFO("X = %f, Y = %f", currentX, currentY);

	camXY = check_boundaries(dX,dY,currentX,currentY);
	dX = camXY.X;
	dY = camXY.Y;
	proportionalControl(dX, dY);
	ROS_INFO("dX = %f, dY = %f",dX,dY);
	float newL1 = sqrt(pow(        (currentX + dX),2.0)                 + pow((maxY - (currentY + dY + gripper_side)),2.0));
	float newL2 = sqrt(pow((maxX - (currentX + dX + gripper_side)),2.0) + pow((maxY - (currentY + dY + gripper_side)),2.0));
	float newL3 = sqrt(pow(        (currentX + dX),2.0)                 + pow(        (currentY + dY),2.0));
	float newL4 = sqrt(pow((maxX - (currentX + dX + gripper_side)),2.0) + pow(        (currentY + dY),2.0));

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

float acceleration_scale(void) {
	if ((camX == 0) && (camY == 0)) {
		maxSpeedRamp = 0.0;
	} else {
		maxSpeedRamp = beta * maxSpeedRamp + (1.0 - beta) * maxSpeed;
		return maxSpeedRamp;  
	}
}

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

		maxSpeedRamp = globalSpeedScalar;
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

void speedThread1(threadStruct s){

	s.srv.request.speed = s.speed;
	if (s.client.call(s.srv)){
		ROS_INFO("Successfully seriviced speed1. \n Speed1 =  %f", s.speed); 
    //flag1 = true;
	} else {
		ROS_ERROR("Failed to call service 1");
    //flag1 = false;
	}
}

void speedThread2(threadStruct s){

	s.srv.request.speed = s.speed;
	if (s.client.call(s.srv)){
		ROS_INFO("Successfully seriviced speed2. \n Speed2 =  %f", s.speed); 
    //flag1 = true;
	} else {
		ROS_ERROR("Failed to call service 2");
    //flag1 = false;
	}
}

void speedThread3(threadStruct s){

	s.srv.request.speed = s.speed;
	if (s.client.call(s.srv)){
		ROS_INFO("Successfully seriviced speed3. \n Speed3 =  %f", s.speed); 
    //flag1 = true;
	} else {
		ROS_ERROR("Failed to call service 3");
    //flag1 = false;
	}
}

void speedThread4(threadStruct s){

	s.srv.request.speed = s.speed;
	if (s.client.call(s.srv)){
		ROS_INFO("Successfully seriviced speed4. \n Speed4 =  %f", s.speed); 
    //flag1 = true;
	} else {
		ROS_ERROR("Failed to call service 4");
    //flag1 = false;
	}
}

void setSpeedSrv(ros::ServiceClient client1, ros::ServiceClient client2, ros::ServiceClient client3, ros::ServiceClient client4,  dynamixel_controllers::SetSpeed srv, speedStruct motorSpeeds){
	threadStruct sSpeedSrv;

	sSpeedSrv.client = client1;
	sSpeedSrv.srv = srv;
	sSpeedSrv.speed = motorSpeeds.s1;

	boost::thread speedSrv1(speedThread1, sSpeedSrv);

	sSpeedSrv.client = client2;
	sSpeedSrv.speed = motorSpeeds.s2;

	boost::thread speedSrv2(speedThread2, sSpeedSrv);


	sSpeedSrv.client = client3;
	sSpeedSrv.speed = motorSpeeds.s3;

	boost::thread speedSrv3(speedThread3, sSpeedSrv);

	sSpeedSrv.client = client4;
	sSpeedSrv.speed = motorSpeeds.s4;

	boost::thread speedSrv4(speedThread4, sSpeedSrv);
}
