
#include <AccelStepper.h>
//#include <PID_v1.h>
#include <ros.h>
#include <geometry_msgs/Point.h>
#include <ros/time.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

//Create accelstepper objects
int motor1_step = 3;
int motor1_direction = 2;

int motor2_step = 4;
int motor2_direction = 5;

int motor3_step = 6;
int motor3_direction = 7;

int motor4_step = 9;
int motor4_direction = 8;


AccelStepper stepper1(1, motor1_step, motor1_direction); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepper2(1, motor2_step, motor2_direction);
AccelStepper stepper3(1, motor3_step, motor3_direction);
AccelStepper stepper4(1, motor4_step, motor4_direction);


/*
  AccelStepper stepper1(1, 3, 2); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
  AccelStepper stepper2(1, 4, 5);
  AccelStepper stepper3(1, 6, 7);
  AccelStepper stepper4(1, 9, 8);
*/
#define ros_everything



////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// VARIABLE DECLERATIONS /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

//stepper speed variables
int stepperMaxSpeed = 600;
float stepperSpeed = 600;

//ROS publish timer stuff
unsigned long previousMillis = 0;  // last time update
long lInterval = 200;              // interval at which to publish ROS messages

float camX, camY, ball_detected = 1;
int steper_counts_1, steper_counts_2, steper_counts_3, steper_counts_4;


//Variables to hold information about the size of the frame, cable lengths etc...
//float maxX = 70.7, maxY = 70.7, current_x = 35, current_y = 35, X, Y;
float maxX = 71.5 - 12, maxY = 89.5 - 12 ;
float maxYstop = maxY;
float minX = 0;
float minY = 0;
float current_x = maxX / 2;
float current_y = maxY / 2;
float X, Y;

float length_1, length_2, length_3, length_4;
float curent_length_1, curent_length_2, curent_length_3, curent_length_4;
float change_in_length_1, change_in_length_2, change_in_length_3, change_in_length_4;

//float scaleX = 70 / 320;
//float scaleY = 70 / 240;

//scaling factors between pixels and steps
float scaleX = maxX / 320;
float scaleY = maxY / 240;
float spoolD = 10;//BagBag
float stepScale =  PI * spoolD / 200;

//starting lengths of cable
float starting_length_1 = 57.28;
float starting_length_2 = 57.28;
float starting_length_3 = 57.28;
float starting_length_4 = 57.28;

//Variables to hold coordinates for end effector
float x, y, z = 1;

//Speed control variables
float maxSpeed1;
float speedMult1, speedMult2, speedMult3, speedMult4;

//Endstop variables
bool endStop = false;
//Used to send endstop status back to host (floats are wasted here, convert to bool messages ASAP!!)
bool endstop1 = false;
bool endstop2 = false;
bool endstop3 = false;
bool endstop4 = false;

//return home variable
bool returnHomeFlag = false;

int direction_tracker_1 = 2;
int direction_tracker_2 = 2;
int direction_tracker_3 = 2;
int direction_tracker_4 = 2;

int step_last_1 = 0;
int step_last_2 = 0;
int step_last_3 = 0;
int step_last_4 = 0;

////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// END OF VARIABLE DECLERATIONS /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// ROS PUBS & SUBS //////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////


//Create ROS nodeHandle
ros::NodeHandle nh;

//Subscribers
#ifdef ros_everything
//Create Point and Quaternion messages (Point used to get ball location from host, quaternion used to publish motor/length data back to host)
geometry_msgs::Quaternion quat_msg;
#endif
geometry_msgs::Point point_msg;

#ifdef ros_everything
//Create publishers
ros::Publisher step_counts("step_counts", &quat_msg);
ros::Publisher lengths_pub("lengths_pub", &quat_msg);
ros::Publisher endstop_publisher("endstop_publisher", &quat_msg);
ros::Publisher direction_tracker("direction_tracker", &quat_msg);
ros::Publisher cam_x_cam_y_current_x_current_y_pub("cam_x_cam_y_current_x_current_y_pub", &quat_msg);

//Create publisher to return gripper positions to hostPC. Create point_msg structure to contain this data
ros::Publisher current_pos("current_pos", &point_msg);
#endif
//Publishers
//ROS subscriber callback function prototypes
void messageCb( const geometry_msgs::Point& data);
ros::Subscriber<geometry_msgs::Point> coordinate_subscriber("coordinate_send_topic", &messageCb);

#ifdef ros_everything

void twistMessageCb( const geometry_msgs::Twist& data);
void speedSetCb( const std_msgs::Int16& data);
void catcherReturnHomeCb( const std_msgs::Bool& data);

//Create ROS subscribers
ros::Subscriber<geometry_msgs::Twist> twist_subscriber("/cmd_vel_mux/input/teleop", &twistMessageCb);
ros::Subscriber<std_msgs::Int16> motor_speed_subscriber("/motor_speed_set", &speedSetCb);
ros::Subscriber<std_msgs::Bool> catcher_return_home("/catcher_return_home", &catcherReturnHomeCb);
#endif
////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// END OF ROS PUBS & SUBS ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

void setup () {

  //Initialize node handler
  nh.initNode();
  //Initialize subscribers & publishers
  nh.subscribe(coordinate_subscriber);
#ifdef ros_everything
  nh.subscribe(twist_subscriber);
  nh.subscribe(motor_speed_subscriber);
  nh.subscribe(catcher_return_home);
  nh.advertise(step_counts);
  nh.advertise(lengths_pub);
  nh.advertise(endstop_publisher);
  nh.advertise(current_pos);
  nh.advertise(direction_tracker);
  nh.advertise(cam_x_cam_y_current_x_current_y_pub);
#endif
  motorSetup();

  //messageCb( const geometry_msgs::Point& data)
  point_msg.x = 0;
  point_msg.y = 120;
  point_msg.z = 160;
  // messageCb(point_msg);
}

void findMax(float d1, float d2, float d3, float d4) {
  d1 = abs(d1);
  d2 = abs(d2);
  d3 = abs(d3);
  d4 = abs(d4);
  maxSpeed1 = max(d1, d2);
  maxSpeed1 = max(maxSpeed1, d3);
  maxSpeed1 = max(maxSpeed1, d4);
}

void setSpeedMult(float maxSpeed, float d1, float d2, float d3, float d4) {
  speedMult1 = d1 / maxSpeed;
  speedMult2 = d2 / maxSpeed;
  speedMult3 = d3 / maxSpeed;
  speedMult4 = d4 / maxSpeed;
}

void setSpeed1(float speedM1, float speedM2, float speedM3, float speedM4) {
  int speed1, speed2, speed3, speed4;
  speed1 = round(stepperSpeed * speedM1);
  speed2 = round(stepperSpeed * speedM2);
  speed3 = round(stepperSpeed * speedM3);
  speed4 = round(stepperSpeed * speedM4);

  stepper1.setSpeed(speed1);
  stepper2.setSpeed(speed2);
  stepper3.setSpeed(speed3);
  stepper4.setSpeed(speed4);


}


void loop() {
#ifdef ros_everything

  //Publish data back to ROS every lInterval milliseconds
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > lInterval) {
    previousMillis = currentMillis;

    //Current gripper position
    point_msg.x = current_x;
    point_msg.y = current_y;
    point_msg.z = 0 ;
    current_pos.publish(&point_msg);

    //current stepper positions
    quat_msg.x = stepper1.currentPosition();
    quat_msg.y = stepper2.currentPosition();
    quat_msg.z = stepper3.currentPosition();
    quat_msg.w = stepper4.currentPosition();
    step_counts.publish(&quat_msg);

    //current cable lengths
    quat_msg.x = length_1;
    quat_msg.y = length_2;
    quat_msg.z = length_3;
    quat_msg.w = length_4;
    lengths_pub.publish(&quat_msg);

    //Tells the user if the endstops are triggered via /endstop_publisher
    endStopCheck();

    quat_msg.x = endstop1;
    quat_msg.y = endstop2;
    quat_msg.z = endstop3;
    quat_msg.w = endstop4;

    endstop_publisher.publish(&quat_msg);

  }

#endif

  endStopCal();
  if ((ball_detected == 0)) {
    //step steppers once per main loop (can this be threaded?)
    // openDrain();

    stepper1.run();
    // openDrain();
    stepper2.run();
    // openDrain();
    stepper3.run();
    // openDrain();
    stepper4.run();
    // openDrain();
    /*

      stepper1.runSpeedToPosition();
      // openDrain();
      stepper2.runSpeedToPosition();
      // openDrain();
      stepper3.runSpeedToPosition();
      // openDrain();
      stepper4.runSpeedToPosition ();
      // openDrain();

    */


  }
  //Do ROS stuff once per main loop
  nh.spinOnce();
}












void moveSteppers(int x, int y) {

  X = scaleX * camX;
  Y = scaleY * camY;

  //find new lengths
  lengthCal(X, Y);

  //find step counts
  steper_counts_1 = length2Step(change_in_length_1);
  steper_counts_2 = length2Step(change_in_length_2);
  steper_counts_3 = length2Step(change_in_length_3);
  steper_counts_4 = length2Step(change_in_length_4);

  //set motor positions
  stepper1.move(steper_counts_1);
  stepper2.move(steper_counts_2);
  stepper3.move(steper_counts_3);
  stepper4.move(steper_counts_4);

}



void lengthCal(float &dX, float &dY) {
  //get current lengths
  curent_length_1 = (stepper1.currentPosition() * stepScale) + starting_length_1;
  curent_length_2 = (stepper2.currentPosition() * stepScale) + starting_length_2;
  curent_length_3 = (stepper3.currentPosition() * stepScale) + starting_length_3;
  curent_length_4 = (stepper4.currentPosition() * stepScale) + starting_length_4;

  //get current (x,y) from lengths
  current_x = maxX / 2 + (sq(curent_length_3) - sq(curent_length_4)) / (2 * maxX);
  current_y = maxY / 2 + (sq(curent_length_4) - sq(curent_length_1)) / (2 * maxY);

  //get new lengths
  length_1 = sqrt(sq(maxX - (current_x + dX)) + sq(maxY - (current_y + dY)));
  length_2 = sqrt(sq(      current_x + dX)  + sq(maxY - (current_y + dY)));
  length_3 = sqrt(sq(      current_x + dX)  + sq(        current_y + dY));
  length_4 = sqrt(sq(maxX - (current_x + dX)) + sq(        current_y + dY));
  length_1 = round(length_1);
  length_2 = round(length_2);
  length_3 = round(length_3);
  length_4 = round(length_4);

  //get dL(change in lengths)
  change_in_length_1 = length_1 - curent_length_1;
  change_in_length_2 = length_2 - curent_length_2;
  change_in_length_3 = length_3 - curent_length_3;
  change_in_length_4 = length_4 - curent_length_4;
}


int length2Step(float &dL) {

  float revs = dL / (PI * spoolD);
  int Step = round(revs * 200);

  return Step;
}


void endStopCal(void) {

  curent_length_1 = ((stepper1.currentPosition()) * stepScale) + starting_length_1;
  curent_length_2 = ((stepper2.currentPosition()) * stepScale) + starting_length_2;
  curent_length_3 = ((stepper3.currentPosition()) * stepScale) + starting_length_3;
  curent_length_4 = ((stepper4.currentPosition()) * stepScale) + starting_length_4;
  //get current (x,y) from lengths
  current_x = maxX / 2 + (sq(curent_length_3) - sq(curent_length_4)) / (2 * maxX);
  current_y = maxY / 2 + (sq(curent_length_4) - sq(curent_length_1)) / (2 * maxY);

  //endStopCheck();

  if (((camX < 0) && (current_x <= minX)) || ((camX > 0) && (current_x >= maxX)) || ((camY < 0) && (current_y <= 0  )) || ((camY > 0) && (current_y >= maxYstop))) {
    endStop = true;
    //   stepper1.moveTo(stepper1.currentPosition());
    //   stepper2.moveTo(stepper2.currentPosition());
    //   stepper3.moveTo(stepper3.currentPosition());
    //   stepper4.moveTo(stepper4.currentPosition());
  }
  else
    endStop = false;
}
int xFlag, yFlag;

//Message callback to populate x, y and z whenever a message is recieved on the coordinate_sender topic
void messageCb( const geometry_msgs::Point& data) {

  quat_msg.x = camX;
  quat_msg.y = camY;
  quat_msg.z = current_x;
  quat_msg.w = current_y;
  cam_x_cam_y_current_x_current_y_pub.publish(&quat_msg);

  //Check whether the X or Y endstops have been hit. If they have, halt that axis
  if ((data.z > 160) && (current_x >= maxX)) {
    camX = 0;    
  } else if ((data.z < 160) && (current_x <= minX)) {
    camX = 0;
  } else {
    camX = maxX * (data.z - 160) / 320;
  }
  //) ? camX = 0.5  : camX = maxX * (data.z - 160) / 320;

  
  if ((data.y > 120) && (current_y <= minY)) {
    camY = 0;
  }
  else if ((data.y < 120) && (current_y >= maxY)) {
    camY = 0;
  } else {
    camY = maxY * (120 - data.y) / 240;
  }

  //? camY = -0.5 : camY = maxY * (120 - data.y) / 240;
  //camY = maxY * (120 - data.y) / 240;
  ball_detected = 0;

  quat_msg.x = camX;
  quat_msg.y = camY;
  quat_msg.z = current_x;
  quat_msg.w = current_y;
  cam_x_cam_y_current_x_current_y_pub.publish(&quat_msg);


  //scale (x,y) co-ordinates
  if ((ball_detected == 0)) { // && (!endStop)) {
    X = camX;
    Y = camY;

    //find new lengths
    lengthCal(X, Y);

    //find step counts
    steper_counts_1 = length2Step(change_in_length_1);
    steper_counts_2 = length2Step(change_in_length_2);
    steper_counts_3 = length2Step(change_in_length_3);
    steper_counts_4 = length2Step(change_in_length_4);

    //set speed
    //findMax(change_in_length_1, change_in_length_2, change_in_length_3, change_in_length_4);
    //setSpeedMult(maxSpeed1, change_in_length_1, change_in_length_2, change_in_length_3, change_in_length_4);
    //setSpeed1(speedMult1, speedMult2, speedMult3, speedMult4);

    //



    quat_msg.x = length_1;
    quat_msg.y = length_2;
    quat_msg.z = length_3;
    quat_msg.w = length_4;
    lengths_pub.publish(&quat_msg);

    //set motor positions
    steper_counts_1 += stepper1.currentPosition();
    steper_counts_2 += stepper2.currentPosition();
    steper_counts_3 += stepper3.currentPosition();
    steper_counts_4 += stepper4.currentPosition();



    steper_counts_1 > step_last_1 ? direction_tracker_1 = 1 : direction_tracker_1 = 0;
    steper_counts_2 > step_last_2 ? direction_tracker_2 = 1 : direction_tracker_2 = 0;
    steper_counts_3 > step_last_3 ? direction_tracker_3 = 1 : direction_tracker_3 = 0;
    steper_counts_4 > step_last_4 ? direction_tracker_4 = 1 : direction_tracker_4 = 0;

    //  steper_counts_1 > step_last_1 ? digitalWrite(motor1_direction, HIGH) : digitalWrite(motor1_direction, LOW);

    quat_msg.x = direction_tracker_1;
    quat_msg.y = direction_tracker_2;
    quat_msg.z = direction_tracker_3;
    quat_msg.w = direction_tracker_4;
    direction_tracker.publish(&quat_msg);

    step_last_1 = steper_counts_1;
    step_last_2 = steper_counts_2;
    step_last_3 = steper_counts_3;
    step_last_4 = steper_counts_4;

    stepper1.moveTo(steper_counts_1);
    stepper2.moveTo(steper_counts_2);
    stepper3.moveTo(steper_counts_3);
    stepper4.moveTo(steper_counts_4);
  }
}

void twistMessageCb( const geometry_msgs::Twist & data) {
  float velocity = data.linear.x * 100;
  float theta = data.angular.z * 10;

  float x_dir = (velocity * cos(theta)) + 160;
  float y_dir = (velocity * sin(theta)) + 120;

  camX = maxX * (x_dir - 160) / 320;
  camY = maxY * (y_dir - 120) / 240;


  //scale (x,y) co-ordinates
  X = camX;
  Y = camY;

  //find new lengths
  lengthCal(X, Y);

  //find step counts
  steper_counts_1 = length2Step(change_in_length_1);
  steper_counts_2 = length2Step(change_in_length_2);
  steper_counts_3 = length2Step(change_in_length_3);
  steper_counts_4 = length2Step(change_in_length_4);

  //set speed
  //findMax(change_in_length_1, change_in_length_2, change_in_length_3, change_in_length_4);
  //setSpeedMult(maxSpeed1, change_in_length_1, change_in_length_2, change_in_length_3, change_in_length_4);
  //setSpeed1(speedMult1, speedMult2, speedMult3, speedMult4);


  //quat_msg.x = steper_counts_1;
  //quat_msg.y = change_in_length_1;

  //quat_msg.z = current_x;
  //quat_msg.w = current_y;

  //set motor positions
  steper_counts_1 += stepper1.currentPosition();
  steper_counts_2 += stepper2.currentPosition();
  steper_counts_3 += stepper3.currentPosition();
  steper_counts_4 += stepper4.currentPosition();
  stepper1.moveTo(steper_counts_1);
  stepper2.moveTo(steper_counts_2);
  stepper3.moveTo(steper_counts_3);
  stepper4.moveTo(steper_counts_4);

}

void speedSetCb( const std_msgs::Int16 & data) {
  stepperMaxSpeed = data.data;
  stepperSpeed = data.data;
  motorSetup();
}






void openDrain() {
  CORE_PIN2_CONFIG = PORT_PCR_ODE | PORT_PCR_MUX(1) | PORT_PCR_SRE | PORT_PCR_DSE ;
  CORE_PIN3_CONFIG = PORT_PCR_ODE | PORT_PCR_MUX(1) | PORT_PCR_SRE | PORT_PCR_DSE ;
  CORE_PIN4_CONFIG = PORT_PCR_ODE | PORT_PCR_MUX(1) | PORT_PCR_SRE | PORT_PCR_DSE ;
  CORE_PIN5_CONFIG = PORT_PCR_ODE | PORT_PCR_MUX(1) | PORT_PCR_SRE | PORT_PCR_DSE ;
  CORE_PIN6_CONFIG = PORT_PCR_ODE | PORT_PCR_MUX(1) | PORT_PCR_SRE | PORT_PCR_DSE ;
  CORE_PIN7_CONFIG = PORT_PCR_ODE | PORT_PCR_MUX(1) | PORT_PCR_SRE | PORT_PCR_DSE ;
  CORE_PIN8_CONFIG = PORT_PCR_ODE | PORT_PCR_MUX(1) | PORT_PCR_SRE | PORT_PCR_DSE ;
  CORE_PIN9_CONFIG = PORT_PCR_ODE | PORT_PCR_MUX(1) | PORT_PCR_SRE | PORT_PCR_DSE ;
}
int pulse_width = 800;

void motorSetup() {

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);


  /*  CORE_PIN2_CONFIG = PORT_PCR_ODE | PORT_PCR_MUX(1) | PORT_PCR_SRE | PORT_PCR_DSE ;
    CORE_PIN3_CONFIG = PORT_PCR_ODE | PORT_PCR_MUX(1) | PORT_PCR_SRE | PORT_PCR_DSE ;
    CORE_PIN4_CONFIG = PORT_PCR_ODE | PORT_PCR_MUX(1) | PORT_PCR_SRE | PORT_PCR_DSE ;
    CORE_PIN5_CONFIG = PORT_PCR_ODE | PORT_PCR_MUX(1) | PORT_PCR_SRE | PORT_PCR_DSE ;
    CORE_PIN6_CONFIG = PORT_PCR_ODE | PORT_PCR_MUX(1) | PORT_PCR_SRE | PORT_PCR_DSE ;
    CORE_PIN7_CONFIG = PORT_PCR_ODE | PORT_PCR_MUX(1) | PORT_PCR_SRE | PORT_PCR_DSE ;
    CORE_PIN8_CONFIG = PORT_PCR_ODE | PORT_PCR_MUX(1) | PORT_PCR_SRE | PORT_PCR_DSE ;
    CORE_PIN9_CONFIG = PORT_PCR_ODE | PORT_PCR_MUX(1) | PORT_PCR_SRE | PORT_PCR_DSE ;
  */
  stepper1.setMinPulseWidth(pulse_width);
  stepper1.setMaxSpeed(stepperMaxSpeed);
  stepper1.setSpeed(stepperSpeed);
  stepper1.setAcceleration(1000);

  stepper2.setMinPulseWidth(pulse_width);
  stepper2.setMaxSpeed(stepperMaxSpeed);
  stepper2.setSpeed(stepperSpeed);
  stepper2.setAcceleration(1000);

  stepper3.setMinPulseWidth(pulse_width);
  stepper3.setMaxSpeed(stepperMaxSpeed);
  stepper3.setSpeed(stepperSpeed);
  stepper3.setAcceleration(1000);

  stepper4.setMinPulseWidth(pulse_width);
  stepper4.setMaxSpeed(stepperMaxSpeed);
  stepper4.setSpeed(stepperSpeed);
  stepper4.setAcceleration(1000);
}


void catcherReturnHomeCb( const std_msgs::Bool & data) {
  returnHome();
}

void returnHome(void) {
  stepper1.moveTo(0);
  stepper2.moveTo(0);
  stepper3.moveTo(0);
  stepper4.moveTo(0);
}


void endStopCheck(void) {
  ((camX < 0) && (current_x <= minX)) ? endstop1 = true : endstop1 = false;
  ((camX > 0) && (current_x >= maxX)) ? endstop2 = true : endstop2 = false;
  ((camY < 0) && (current_y <= minY)) ? endstop3 = true : endstop3 = false;
  ((camY > 0) && (current_y >= maxY)) ? endstop4 = true : endstop4 = false;
}

