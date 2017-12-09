#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "dynamixel_msgs/JointState.h"
#include <math.h>
#include "dynamixel_controllers/SetSpeed.h"
#include "boost/thread.hpp"
#include "boost/chrono.hpp"

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

const int hyst = 0;
const int tolerance = 0;

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




bool goFlag = false;
float m1Position,m2Position,m3Position,m4Position;

void cameraDataCallback(const geometry_msgs::Point::ConstPtr& coordinate_msg){
  dataX = 89.5*(coordinate_msg->x-320)/640.0;
  dataY = 71.5*(coordinate_msg->y-240)/480.0;
  ball_detected = coordinate_msg->z;
  goFlag = true;
 // ball_detected == 1.0 ? goFlag = false : goFlag = true;
  

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

// float checkAxisBounds(float cam, float current, float minimum, float maximum) {
//   if ((current <= minimum) && (cam < 0)) {
//     boundFlag = true;
//     return (minimum - current - hyst);
//   } else if ((current >= maximum) && (cam > 0)) {
//     boundFlag = true;
//     return (maximum - current + hyst);
//   } else {
//     boundFlag = false;
//     return cam;
//   }
// }

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
  camXY.X = checkAxisDeadBand(camX, 89.5*tolerance/640);
  camXY.Y = checkAxisDeadBand(camY, 71.5*tolerance/480);
  return camXY;
}

lengthStruct findNewLengths(float dX, float dY){
  lengthStruct temp;
  camVals camXY;
  float m1Length = motorPositionToLength(m1Position);
  float m2Length = motorPositionToLength(m2Position);
  float m3Length = motorPositionToLength(m3Position);
  float m4Length = motorPositionToLength(m4Position);
  float currentl1 = m1Length + startingl1;
  float currentl2 = m2Length + startingl2;
  float currentl3 = m3Length + startingl3;
  float currentl4 = m4Length + startingl4;

  float currentX = maxX / 2.0 + (pow(currentl1,2.0) - pow(currentl2,2.0)) / (2 * maxX);
  float currentY = maxY / 2.0 + (pow(currentl4,2.0) - pow(currentl2,2.0)) / (2 * maxY);
  //l3^2 = sqrt(x^2 + y^2)
  //float currentY = maxY - sqrt(pow(currentl1,2.0) - pow(currentX,2.0));
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
  temp.nl1 = newL1;
  temp.nl2 = newL2;
  temp.nl3 = newL3;
  temp.nl4 = newL4;
  temp.x = currentX;
  temp.y = currentY;
  return temp;
}


float maxSpeedRamp = 0.0;
float maxSpeed = 10.0;
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
  maxSpeedRamp = 10.0;
  speeds.s1 = (maxSpeedRamp * speedMult1);
  speeds.s2 = (maxSpeedRamp * speedMult2);
  speeds.s3 = (maxSpeedRamp * speedMult3);
  speeds.s4 = (maxSpeedRamp * speedMult4);
  }
  else{
  speeds.s1 = 0.0;
  speeds.s2 = 0.0;
  speeds.s3 = 0.0;
  speeds.s4 = 0.0;
  }

  return speeds;
}

struct threadStruct{
  ros::ServiceClient client;
  dynamixel_controllers::SetSpeed srv;
  float speed;
};

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

      // srv.request.speed = motorSpeeds.s1;
      // if (client1.call(srv)){
      //   flag1 = true;
      // } else {
      //   ROS_ERROR("Failed to call service 1");
      //   flag1 = false;
      // }
      // srv.request.speed = motorSpeeds.s2;
      // if (client2.call(srv)){
      //   flag2 = true;
      // } else {
      //   ROS_ERROR("Failed to call service 2");
      //   flag2 = false;
      // }

      // srv.request.speed = motorSpeeds.s3;
      // if (client3.call(srv)){
      //   flag3 = true;
      // } else {
      //   ROS_ERROR("Failed to call service 3");
      //   flag3 = false;
      // }

      // srv.request.speed = motorSpeeds.s4;
      // if (client4.call(srv)){
      //   flag4 = true;

      // } else {
      //   ROS_ERROR("Failed to call service 4");
      //   flag4 = false;
      // }
      // if (flag1 && flag2 && flag3 && flag4){
      //   ROS_INFO("Successfully published all speeds. \n Speed1 =  %f Speed2 =  %f Speed3 =  %f Speed4 =  %f \n", motorSpeeds.s1, motorSpeeds.s2, motorSpeeds.s3, motorSpeeds.s4);     
      // }
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
  std_msgs::Bool bound_flag_obj;
  // dynamixel_msgs::JointState motor1sim;
  // dynamixel_msgs::JointState motor2sim;
  // dynamixel_msgs::JointState motor3sim;
  // dynamixel_msgs::JointState motor4sim;
  geometry_msgs::Point XY;
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
  ros::Publisher bound_flag_pub = n.advertise<std_msgs::Bool>("/bound_flag", 1);
  
  // ros::Publisher motor1simPub = n.advertise<dynamixel_msgs::JointState>("/motor1_controller/state", 1);
  // ros::Publisher motor2simPub = n.advertise<dynamixel_msgs::JointState>("/motor2_controller/state", 1);
  // ros::Publisher motor3simPub = n.advertise<dynamixel_msgs::JointState>("/motor3_controller/state", 1);
  // ros::Publisher motor4simPub = n.advertise<dynamixel_msgs::JointState>("/motor4_controller/state", 1);

  ros::Publisher motorLengthPub = n.advertise<geometry_msgs::Quaternion>("motorLengths", 1);
  ros::Publisher motorPositionsPub = n.advertise<geometry_msgs::Quaternion>("motorPositions", 1);
  ros::Publisher PositionPub = n.advertise<geometry_msgs::Point>("current_pos", 1);
  speedStruct motorSpeeds;

  ros::Rate loop_rate(250);
  lengthStruct newLengths;
  bool initFlag = true;
  while (ros::ok()){
    // goFlag = false;
    if (goFlag == true){
    	if (ball_detected == 1.0){
    		dataX = 0.0;
    		dataY = 0.0;
    	}
      newLengths = findNewLengths(dataX, dataY);
      motorSpeeds = calculate_speeds(newLengths);
      bound_flag_obj.data = boundFlag;
      bound_flag_pub.publish(bound_flag_obj);
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
      //PositionPub.publish(XY);
      //motorLengthPub.publish(lengths);
      motorPositionsPub.publish(positions);

      setSpeedSrv(client1,client2,client3,client4,srv,motorSpeeds);

      m1NewPos.data = motorLengthToPosition(newLengths.l1) + m1Position;
      m2NewPos.data = motorLengthToPosition(newLengths.l2) + m2Position;
      m3NewPos.data = motorLengthToPosition(newLengths.l3) + m3Position;
      m4NewPos.data = motorLengthToPosition(newLengths.l4) + m4Position;
      // motor1sim.current_pos = m1NewPos.data;
      // motor2sim.current_pos = m2NewPos.data;
      // motor3sim.current_pos = m3NewPos.data;
      // motor4sim.current_pos = m4NewPos.data;

      motor1Pub.publish(m1NewPos);
      motor2Pub.publish(m2NewPos);
      motor3Pub.publish(m3NewPos);
      motor4Pub.publish(m4NewPos);

      // motor1simPub.publish(motor1sim);
      // motor2simPub.publish(motor2sim);
      // motor3simPub.publish(motor3sim);
      // motor4simPub.publish(motor4sim);
      initFlag = false;
      goFlag = false;
    }
    else if (initFlag == true){
      m1NewPos.data = PI;
      m2NewPos.data = PI;
      m3NewPos.data = PI;
      m4NewPos.data = PI;
      motorSpeeds.s1 = 10.0;
      motorSpeeds.s2 = 10.0;
      motorSpeeds.s3 = 10.0;
      motorSpeeds.s4 = 10.0;
      setSpeedSrv(client1,client2,client3,client4,srv,motorSpeeds);
      motor1Pub.publish(m1NewPos);
      motor2Pub.publish(m2NewPos);
      motor3Pub.publish(m3NewPos);
      motor4Pub.publish(m4NewPos);

      ros::spinOnce();
      loop_rate.sleep();
    }else if (initFlag == false){
      newLengths = findNewLengths(0, 0);
      motorSpeeds = calculate_speeds(newLengths);
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
     }
    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
}