#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"
#include "dynamixel_msgs/JointState.h"
#include <math.h>

int camX = 320;
int camY = 240;
float dataX = camX;
float dataY = camY;
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
};
volatile bool goFlag = false;
float m1Position,m2Position,m3Position,m4Position;

void cameraDataCallback(const geometry_msgs::Point::ConstPtr& coordinate_msg){
	dataX = coordinate_msg->x;
	dataY = coordinate_msg->y;
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
  rotations = (motorPositionScale * motorPosition/360);
  length = rotations * PI * spoolD;
  return length;
}

float motorLengthToPosition(float length){
  float motorPosition;
  float rotations;
  rotations = length/(PI*spoolD);
  motorPosition = 360*rotations/motorPositionScale;
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

  float currentX = maxX / 2 + (pow(currentl3,2.0) - pow(currentl4,2.0)) / (2 * maxX);
  float currentY = maxY / 2 + (pow(currentl4,2.0) - pow(currentl2,2.0)) / (2 * maxY);

  camXY = check_boundaries(dX,dY,currentX,currentY);
  dX = camXY.X;
  dY = camXY.Y;

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




int main(int argc, char **argv)
{
	ros::init(argc, argv, "kinematic_controller");
	ros::NodeHandle n;
  std_msgs::Float64 m1NewPos;
  std_msgs::Float64 m2NewPos;
  std_msgs::Float64 m3NewPos;
  std_msgs::Float64 m4NewPos;
	ros::Subscriber cameraSub = n.subscribe("coordinate_send_topic", 1, cameraDataCallback);
  ros::Subscriber motor1Sub = n.subscribe("/motor1_controller/state", 1, m1Callback);
  ros::Subscriber motor2Sub = n.subscribe("/motor2_controller/state", 1, m1Callback);
  ros::Subscriber motor3Sub = n.subscribe("/motor3_controller/state", 1, m1Callback);
  ros::Subscriber motor4Sub = n.subscribe("/motor4_controller/state", 1, m1Callback);
  ros::Publisher motor1Pub = n.advertise<std_msgs::Float64>("/motor1_controller/command", 1);
  ros::Publisher motor2Pub = n.advertise<std_msgs::Float64>("/motor2_controller/command", 1);
  ros::Publisher motor3Pub = n.advertise<std_msgs::Float64>("/motor3_controller/command", 1);
  ros::Publisher motor4Pub = n.advertise<std_msgs::Float64>("/motor4_controller/command", 1);
  ros::Rate loop_rate(10);
  lengthStruct newLengths;




	while (ros::ok()){
    if (goFlag = true){

      newLengths = findNewLengths(dataX, dataY);

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
