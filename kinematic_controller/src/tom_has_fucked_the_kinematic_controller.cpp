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
float dataX = camX;
float dataY = camY;
float maxX = 89.5;
float maxY = 71.5;
int ball_detected = 0;
int motor_acceleration = 2000;
int stepperMaxSpeed = 300;
int resolution_divider = 4;
//range from 0 to 2*pi

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
	dataX = coordinate_msg->x;
	dataY = coordinate_msg->y;
	dataX = dataX - 320;
	dataY = dataY - 240;
	ball_detected = coordinate_msg->z;
	if (ball_detected == 1){
		goFlag = false;
	} else {
		goFlag = true;
	}

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
	//motorPositionScale = 180/pi
	rotations = (motorPositionScale * (motorPosition - PI)/360);
	rotations = (rotations*4) - PI;
	length = rotations * PI * spoolD;

	return length;
}

float motorLengthToPosition(float length){
	float motorPosition;
	float rotations;
	rotations = length/(PI*spoolD);
	motorPosition = 360*rotations/motorPositionScale;
	motorPosition = (motorPosition/4) ;
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
		return 0;
	} else {
		deadFlag = false;
		return location;
	}
}

struct camVals{
	float X;
	float Y;
};

struct gripperLocation{
	float X;
	float Y;
};

struct current_lengths_s{
	float x;
	float y;
	float z;
	float w;
};

struct change_in_lengths_s{
	float x;
	float y;
	float z;
	float w;
};

gripperLocation GripperLocation;
current_lengths_s Current_lengths_s;
camVals check_boundaries(float camX, float camY, float currentX, float currentY) {
	camVals camXY;
	camX = checkAxisBounds(camX, currentX, (10), (maxX - 10));
	camY = checkAxisBounds(camY, currentY, (10), (maxY - 10));
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
	Current_lengths_s.x = currentl1;
	Current_lengths_s.y = currentl2;
	Current_lengths_s.z = currentl3;
	Current_lengths_s.w = currentl4;

	float currentX = maxX / 2 + (pow(currentl3,2.0) - pow(currentl4,2.0)) / (2 * maxX);
	float currentY = maxY / 2 + (pow(currentl4,2.0) - pow(currentl2,2.0)) / (2 * maxY);
	GripperLocation.X = currentX;
	GripperLocation.Y = currentY;
	//camXY = check_boundaries(dX,dY,currentX,currentY);
	//dX = camXY.X;
	//dY = camXY.Y;

	float newL1 = sqrt(pow(      (currentX + dX),2.0)  + pow((maxY - (currentY + dY)),2.0));
	float newL2 = sqrt(pow((maxX - (currentX + dX)),2.0) + pow((maxY - (currentY + dY)),2.0));
	float newL3 = sqrt(pow(      (currentX + dX),2.0)  + pow(        (currentY + dY),2.0));
	float newL4 = sqrt(pow((maxX - (currentX + dX)),2.0) + pow(        (currentY + dY),2.0));

	temp.l1 = newL1 - currentl1;
	temp.l2 = newL2 - currentl2;
	temp.l3 = newL3 - currentl3;
	temp.l4 = newL4 - currentl4;
	return temp;
}


float maxSpeedRamp = 0;
int maxSpeed = 6;
float beta = 0.9999;
float acceleration_scale(void) {
	if ((camX == 0) && (camY == 0)) {
		maxSpeedRamp = 0;
	} else {
		maxSpeedRamp = beta * maxSpeedRamp + (1 - beta) * maxSpeed;
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
	float speedMult1 = change_in_lengths.l1 / speedScaler;
	float speedMult2 = change_in_lengths.l2 / speedScaler;
	float speedMult3 = change_in_lengths.l3 / speedScaler;
	float speedMult4 = change_in_lengths.l4 / speedScaler;

	//maxSpeedRamp = acceleration_scale();
	maxSpeedRamp = 40;
	speeds.s1 = round(maxSpeedRamp * speedMult1);
	speeds.s2 = round(maxSpeedRamp * speedMult2);
	speeds.s3 = round(maxSpeedRamp * speedMult3);
	speeds.s4 = round(maxSpeedRamp * speedMult4);
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
	geometry_msgs::Point current_pos;
	geometry_msgs::Point dataXYpub;
	geometry_msgs::Quaternion current_lengthsQ;
	geometry_msgs::Quaternion speed_pub;
	geometry_msgs::Quaternion dl_pub;
	geometry_msgs::Quaternion motor_positions;
	ros::Subscriber cameraSub = n.subscribe("coordinate_send_topic", 1, cameraDataCallback);
	ros::Subscriber motor1Sub = n.subscribe("/motor1_controller/state", 1, m1Callback);
	ros::Subscriber motor2Sub = n.subscribe("/motor2_controller/state", 1, m2Callback);
	ros::Subscriber motor3Sub = n.subscribe("/motor3_controller/state", 1, m3Callback);
	ros::Subscriber motor4Sub = n.subscribe("/motor4_controller/state", 1, m4Callback);
	ros::Publisher motor1Pub = n.advertise<std_msgs::Float64>("/motor1_controller/command", 1);
	ros::Publisher motor2Pub = n.advertise<std_msgs::Float64>("/motor2_controller/command", 1);
	ros::Publisher motor3Pub = n.advertise<std_msgs::Float64>("/motor3_controller/command", 1);
	ros::Publisher motor4Pub = n.advertise<std_msgs::Float64>("/motor4_controller/command", 1);
	ros::Publisher current_position = n.advertise<geometry_msgs::Point>("/current_pos", 1);
	ros::Publisher current_lengths = n.advertise<geometry_msgs::Quaternion>("/current_lengths", 1);
	ros::Publisher speed_publisher = n.advertise<geometry_msgs::Quaternion>("/motor_speeds", 1);
	ros::Publisher change_in_len_pub = n.advertise<geometry_msgs::Quaternion>("/change_in_lengths", 1);
	ros::Publisher dataXYpublisher = n.advertise<geometry_msgs::Point>("/dataXY", 1);
	ros::Publisher motor_positions_in = n.advertise<geometry_msgs::Quaternion>("/motor_positions_in", 1);
	ros::Publisher motor_positions_out = n.advertise<geometry_msgs::Quaternion>("/motor_positions_out", 1);
	
	ros::Rate loop_rate(10);
	lengthStruct newLengths;

	m1NewPos.data = PI;
	m2NewPos.data = PI;
	m3NewPos.data = PI;
	m4NewPos.data = PI;
	motor1Pub.publish(m1NewPos);
	ros::spinOnce();

	motor2Pub.publish(m2NewPos);
	ros::spinOnce();

	motor3Pub.publish(m3NewPos);
	ros::spinOnce();

	motor4Pub.publish(m4NewPos);
	ros::spinOnce();
	ros::spinOnce();

	speedStruct motorSpeeds;

	motorSpeeds.s1 = 5;
	motorSpeeds.s2 = 5;
	motorSpeeds.s3 = 5;
	motorSpeeds.s4 = 5;


	srv.request.speed = motorSpeeds.s1;
	if (client1.call(srv)){
		ROS_INFO("Success %f", motorSpeeds.s1);			
	} else {
		ROS_ERROR("Failed to call service 1");
	}

	srv.request.speed = motorSpeeds.s2;
	if (client2.call(srv)){
		ROS_INFO("Success %f", motorSpeeds.s2);			
	} else {
		ROS_ERROR("Failed to call service 2");
	}

	srv.request.speed = motorSpeeds.s3;
	if (client3.call(srv)){
		ROS_INFO("Success %f", motorSpeeds.s3);			
	} else {
		ROS_ERROR("Failed to call service 3");
	}

	srv.request.speed = motorSpeeds.s4;
	if (client4.call(srv)){
		ROS_INFO("Success %f", motorSpeeds.s4);			
	} else {
		ROS_ERROR("Failed to call service 4");
	}


	int microseconds = 5*100*1000;
	usleep(microseconds);

	motor1Pub.publish(m1NewPos);
	ros::spinOnce();

	motor2Pub.publish(m2NewPos);
	ros::spinOnce();

	motor3Pub.publish(m3NewPos);
	ros::spinOnce();

	motor4Pub.publish(m4NewPos);
	ros::spinOnce();
	ros::spinOnce();

	usleep(microseconds*4);
	ros::spinOnce();
	usleep(microseconds);
	ros::spinOnce();
	usleep(microseconds);


	while (ros::ok()){
		if (goFlag = true){

			newLengths = findNewLengths(dataX, dataY);
			dataXYpub.x = dataX;
			dataXYpub.y = dataY;
			dataXYpublisher.publish(dataXYpub);
			motorSpeeds = calculate_speeds(newLengths);
			speed_pub.x = motorSpeeds.s1;
			speed_pub.y = motorSpeeds.s2;
			speed_pub.z = motorSpeeds.s3;
			speed_pub.w = motorSpeeds.s4;
			speed_publisher.publish(speed_pub);

			current_pos.x = GripperLocation.X;
			current_pos.y = GripperLocation.Y;
			current_position.publish(current_pos);

			current_lengthsQ.x = Current_lengths_s.x;
			current_lengthsQ.y = Current_lengths_s.y;
			current_lengthsQ.z = Current_lengths_s.z;
			current_lengthsQ.w = Current_lengths_s.w;
			current_lengths.publish(current_lengthsQ);

			dl_pub.x = newLengths.l1;
			dl_pub.y = newLengths.l2;
			dl_pub.z = newLengths.l3;
			dl_pub.w = newLengths.l4;
			change_in_len_pub.publish(dl_pub);



			motorSpeeds.s1 = 5;
			motorSpeeds.s2 = 5;
			motorSpeeds.s3 = 5;
			motorSpeeds.s4 = 5;

			
			srv.request.speed = motorSpeeds.s1;
			if (client1.call(srv)){
				ROS_INFO("Success %f", motorSpeeds.s1);			
			} else {
				ROS_ERROR("Failed to call service 1");
			}

			srv.request.speed = motorSpeeds.s2;
			if (client2.call(srv)){
				ROS_INFO("Success %f", motorSpeeds.s2);			
			} else {
				ROS_ERROR("Failed to call service 2");
			}

			srv.request.speed = motorSpeeds.s3;
			if (client3.call(srv)){
				ROS_INFO("Success %f", motorSpeeds.s3);			
			} else {
				ROS_ERROR("Failed to call service 3");
			}

			srv.request.speed = motorSpeeds.s4;
			if (client4.call(srv)){
				ROS_INFO("Success %f", motorSpeeds.s4);			
			} else {
				ROS_ERROR("Failed to call service 4");
			}


			motor_positions.x = m1Position;
			motor_positions.y = m2Position;
			motor_positions.z = m3Position;
			motor_positions.w = m4Position;
			motor_positions_in.publish(motor_positions);

			m1NewPos.data = m1Position + motorLengthToPosition(newLengths.l1);
			m2NewPos.data = m2Position + motorLengthToPosition(newLengths.l2);
			m3NewPos.data = m3Position + motorLengthToPosition(newLengths.l3);
			m4NewPos.data = m4Position + motorLengthToPosition(newLengths.l4);

			motor_positions.x = m1NewPos.data;
			motor_positions.y = m2NewPos.data;
			motor_positions.z = m3NewPos.data;
			motor_positions.w = m4NewPos.data;
			motor_positions_out.publish(motor_positions);

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
