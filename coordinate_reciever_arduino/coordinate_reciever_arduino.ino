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
}
  
