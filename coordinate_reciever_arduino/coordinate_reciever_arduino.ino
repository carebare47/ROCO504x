#include <AccelStepper.h>
#include <PID_v1.h>

/*
   Arduino rosserial coordinate reciever
   Written by Tom Queen with Arduino 1.0.6 and ROS Indigo
*/

#include <ros.h>
#include <geometry_msgs/Point.h>
#include <ros/time.h>
//#define PIDon
#define relative
//Create node handle
ros::NodeHandle nh;

#define stepperSpeed 1200
#define total_x_steps 500
#define total_y_steps 500

//AccelStepper stepper1(1, 9, 8); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
//AccelStepper stepper2(1, 11, 10);
//AccelStepper stepper3(1, 13, 12);
//AccelStepper stepper4(1, 6, 7);



AccelStepper stepper1(1, 4, 5); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepper2(1, 6, 7);
AccelStepper stepper3(1, 8, 9);
AccelStepper stepper4(1, 3, 2);

double xSetpoint = 160;
double ySetpoint = 120;

double xInput, xOutput;
double yInput, yOutput;
double ki = 0;
double kp = 1;
double kd = 0;

float Pos1;


#ifdef PIDon
PID xPID(&xInput, &xOutput, &xSetpoint, ki, kp, kd, DIRECT);
PID yPID(&yInput, &yOutput, &ySetpoint, ki, kp, kd, DIRECT);
#endif
void FindPos(int t1, int t2, int t3, int t4) {

  Pos1 = (PI * 10) * (t1 / 200);

}


geometry_msgs::Point point_msg;
ros::Publisher current_pos("current_pos", &point_msg);

//Variables to hold coordinates for end effector
float x, y, z;

//Message callback to populate x, y and z whenever a message is recieved on the coordinate_sender topic
void messageCb( const geometry_msgs::Point& data) {
  x = data.x;
  y = data.y;
  z = data.z;

  xInput = x - 160;
  yInput = y - 120;

}

//Create subscriber to coordinate_sender topic
ros::Subscriber<geometry_msgs::Point> coordinate_subscriber("coordinate_send_topic", &messageCb);

void setup () {
  nh.advertise(current_pos);

  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);

  digitalWrite(A5, HIGH);
  digitalWrite(A4, HIGH);
  digitalWrite(A3, HIGH);


  motorSetup();
  xInput = x;
  yInput = y;

  //Turn on PID
#ifdef PIDon
  xPID.SetMode(AUTOMATIC);
  yPID.SetMode(AUTOMATIC);
#endif
  //Initialize node handler
  nh.initNode();
  //Initialize subscriber
  nh.subscribe(coordinate_subscriber);
}


void loop () {

#ifdef PIDon
  xPID.Compute();
  yPID.Compute();
#endif

#ifdef PIDon
  motorController(xOutput, yOutput);
#else
  motorController(xInput, yInput);
#endif

  //if (stepper1.currentPosition() >

  if (z == 0) {
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();
  }
  //Do ROS stuff once per main loop
  nh.spinOnce();

}

void motorController(int x, int y) {
  //Do something with the recieved coordinates

  /*  int m1 = y - x;
    int m2 = y + x;
    int m3 = -y - x;
    int m4 = -y + x;

    m1 > 0 ? stepper1.moveTo(400) : stepper1.moveTo(-400);
    m2 > 0 ? stepper2.moveTo(400) : stepper2.moveTo(-400);
    m3 > 0 ? stepper3.moveTo(400) : stepper3.moveTo(-400);
    m4 > 0 ? stepper4.moveTo(400) : stepper4.moveTo(-400);
  */


  //  x, y in        -->  stepper 1-4 out
  //  stepper1 = y - x
  //  stepper2 = y + x
  //  stepper3 = -y - x
  //  stepper4 = -y + x

  // camera 320 x 240
  // frame  600 x 600


  //  stepper 1-4 in -->  x,y out
  //

  // stp1 = (y - x)
  // stp2 = (y + x)
  // stp1 + stp2 = y - x + y + x = 2y
  // stp2 - stp1 = (y + x) - (y - x) = y + x - y + x = 2x
  //  y = (stp1 + stp2) / 2
  //  x = (stp2 - stp1) / 2

  int current_y = (stepper1.currentPosition() + stepper2.currentPosition()) / 2;
  int current_x = (stepper2.currentPosition() - stepper1.currentPosition()) / 2;


  if ((current_y > 250) && (y > 0)) {
    y = 0;
  } else if ((current_y < -250) && (y < 0)) {
    y = 0;
  }

  if ((current_x > 250) && (x > 0)) {
    x = 0;
  } else if ((current_x < -250)&& (x < 0)) {
    x = 0;
  }

      point_msg.x = current_x;
      point_msg.y = current_y;
    current_pos.publish( &point_msg);

 /* if (current_y > ((total_y_steps)/2) {
    y = 0;
  } else if (current_y < (((total_y_steps)/(-2))) {
    y = 0;
  }

  if (current_x > 300) {
    x = 0;
  } else if (current_x < -300) {
    x = 0;
  }
*/


#ifdef relative
  stepper1.move(y - x);
  stepper2.move(y + x);
  stepper3.move(-y - x);
  stepper4.move(-y + x);
#endif
#ifdef absolute
  stepper1.moveTo(y - x);
  stepper2.moveTo(y + x);
  stepper3.moveTo(-y - x);
  stepper4.moveTo(-y + x);
#endif

}

void motorSetup() {
  stepper1.setMinPulseWidth(200);
  stepper1.setMaxSpeed(1350);
  stepper1.setSpeed(stepperSpeed);
  stepper1.setAcceleration(10000);

  stepper2.setMinPulseWidth(200);
  stepper2.setMaxSpeed(1350);
  stepper2.setSpeed(stepperSpeed);
  stepper2.setAcceleration(10000);

  stepper3.setMinPulseWidth(200);
  stepper3.setMaxSpeed(1350);
  stepper3.setSpeed(stepperSpeed);
  stepper3.setAcceleration(10000);

  stepper4.setMinPulseWidth(200);
  stepper4.setMaxSpeed(1350);
  stepper4.setSpeed(stepperSpeed);
  stepper4.setAcceleration(10000);
}
