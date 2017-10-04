#include <PID_v1.h>

/*
 * Arduino rosserial coordinate reciever
 * Written by Tom Queen with Arduino 1.0.6 and ROS Indigo
 */

#include <ros.h>
#include <geometry_msgs/Point.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

//Create node handle
ros::NodeHandle nh;


#define motor_A_1 2
#define motor_A_2 4
#define motor_A_EN 3

#define motor_B_1 7
#define motor_B_2 6
#define motor_B_EN 5

#define motor_C_1 8
#define motor_C_2 9
#define motor_C_EN 10

#define motor_D_1 13
#define motor_D_2 12
#define motor_D_EN 11


//Variables to hold coordinates for end effector
float x, y, z;

//Message callback to populate x, y and z whenever a message is recieved on the coordinate_sender topic
void messageCb( const geometry_msgs::Point& data){
  x = data.x;
  y = data.y;
  z = data.z;
}

//Create subscriber to coordinate_sender topic
ros::Subscriber<geometry_msgs::Point> coordinate_subscriber("coordinate_sender", &messageCb);

void setup () {
  motorSetup();
  
  //Initialize node handler
  nh.initNode();
  //Initialize subscriber
  nh.subscribe(coordinate_subscriber);
}


void loop () {
  
  
  motorController(x, y);
  //Do ROS stuff once per main loop
  nh.spinOnce();
  
}

void motorController(int x, int y){
  //Do something with the recieved coordinates
  
  //assuming point x will be between 0 and 320 & point y between 0 and 240
  if (x < 160){
      direction_left();  
}

void motorSetup(){
 pinMode(motor_A_1, OUTPUT);
 pinMode(motor_A_2, OUTPUT);
 pinMode(motor_A_EN, OUTPUT);
 
 pinMode(motor_B_1, OUTPUT);
 pinMode(motor_B_2, OUTPUT);
 pinMode(motor_B_EN, OUTPUT);
 
 pinMode(motor_C_1, OUTPUT);
 pinMode(motor_C_2, OUTPUT);
 pinMode(motor_C_EN, OUTPUT);
 
 pinMode(motor_D_1, OUTPUT);
 pinMode(motor_D_2, OUTPUT);
 pinMode(motor_D_EN, OUTPUT);
 
}


void set_direction_left(){
  A_forwards();
  C_forwards();
  
  B_reverse();
  D_reverse();
}




void A_forwards(){
  digitalWrite(motor_A_1, HIGH);
  digitalWrite(motor_A_2, LOW);
}

void A_reverse(){
  digitalWrite(motor_A_2, HIGH);
  digitalWrite(motor_A_1, LOW);
}


void B_forwards(){
  digitalWrite(motor_B_1, HIGH);
  digitalWrite(motor_B_2, LOW);
}

void B_reverse(){
  digitalWrite(motor_B_2, HIGH);
  digitalWrite(motor_B_1, LOW);
}

void C_forwards(){
  digitalWrite(motor_C_1, HIGH);
  digitalWrite(motor_C_2, LOW);
}

void C_reverse(){
  digitalWrite(motor_C_2, HIGH);
  digitalWrite(motor_C_1, LOW);
}


void D_forwards(){
  digitalWrite(motor_D_1, HIGH);
  digitalWrite(motor_D_2, LOW);
}

void D_reverse(){
  digitalWrite(motor_D_2, HIGH);
  digitalWrite(motor_D_1, LOW);
}


  

