#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "dynamixel_msgs/JointState.h"

float camX = 320;
float camY = 240;
float dataX = camX;
float dataY = camY;
int ball_detected = 0;
float change_in_length_1, change_in_length_2, change_in_length_3, change_in_length_4;

int motor_acceleration = 2000;
int stepperMaxSpeed = 300;


float PI = 3.14159265359;
float spoolD = 10;
float motorPositionScale = 180/PI;
//starting lengths of cable
float startingl1 = 57.27674048;
float startingl2 = 57.27674048;
float startingl3 = 57.27674048;
float startingl4 = 57.27674048;

struct lengthStruct{
  float l1;
  float l2;
  float l3;
  float l4;
};

float m1Position,m2Position,m3Position,m4Position;
void perform_calculations(void);

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
//	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void cameraDataCallback(const geometry_msgs::Point::ConstPtr& coordinate_msg){
	dataX = coordinate_msg->data.x;
	dataY = coordinate_msg->data.y;
  ball_detected = coordinate_msg->data.z;
}

void m1Callback(const dynamixel_msgs::JointState::ConstPtr& m1State){
  m1Position = m1State->data.current_pos;
}

float motorPositionToLength(float motorPosition){
  float length;
  float rotations;
  rotations = (motorPositionScale * motorPosition/360);
  length = rotations * PI * spoolD;
  return length;
}

 lengthStruct findNewLengths(float l1, float l2, float l3, float l4){
  lengthStruct temp;
  float currentl1 = startingl1 + l1;
  float currentl2 = startingl2 + l2;
  float currentl3 = startingl3 + l3;
  float currentl4 = startingl4 + l4;

  float currentX = maxX / 2 + (sq(current_length_3) - sq(current_length_4)) / (2 * maxX);
  float currentY = maxY / 2 + (sq(current_length_4) - sq(current_length_2)) / (2 * maxY);

  float newL1 = sqrt(sq(      current_x + dX)  + sq(maxY - (current_y + dY)));
  float newL2 = sqrt(sq(maxX - (current_x + dX)) + sq(maxY - (current_y + dY)));
  float newL3 = sqrt(sq(      current_x + dX)  + sq(        current_y + dY));
  float newL4 = sqrt(sq(maxX - (current_x + dX)) + sq(        current_y + dY));

  temp.l1 = newL1 - currentl1;
  temp.l2 = newL1 - currentl2;
  temp.l3 = newL1 - currentl3;
  temp.l4 = newL1 - currentl4;
  return temp;
}




int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
	ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
	ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
	ros::Subscriber cameraSub = n.subscribe("coordinate_send_topic", 1, cameraDataCallback);
  ros::Subscriber motor1Sub = n.subscribe("/motor1_controller/state", 1, m1Callback);
  ros::Subscriber motor2Sub = n.subscribe("/motor2_controller/state", 1, m1Callback);
  ros::Subscriber motor3Sub = n.subscribe("/motor3_controller/state", 1, m1Callback);
  ros::Subscriber motor4Sub = n.subscribe("/motor4_controller/state", 1, m1Callback);
  

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.


   */

	while (ros::ok()){
	ros::spinOnce();


	}
	//	perform_calculations();
	//endStopCheck2();


return 0;
}
