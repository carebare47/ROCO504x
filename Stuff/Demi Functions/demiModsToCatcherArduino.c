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
int stepper_counts_1, stepper_counts_2, stepper_counts_3, stepper_counts_4;


//Variables to hold information about the size of the frame, cable lengths etc...
//float maxX = 70.7, maxY = 70.7, current_x = 35, current_y = 35, X, Y;

// Frame measurements. Square gripper assumption leads to point gripper assumption if offsets applied.
const float gripper_offset = 12.0;
const float maxX = 71.5 - gripper_offset;
const float maxY = 89.5 - gripper_offset;
const float minX =  0.0 + gripper_offset;
const float minY =  0.0 + gripper_offset;

// Useful values from min/max x/y:
const float distX = maxX - minX;
const float distY = maxY - minY;
const float midX = (maxX - minX) / 2;
const float midY = (maxY - minY) / 2;

// Other global variables
float xBorder = 10.0;
float yBorder = 10.0;
float current_x = midX;
float current_y = midY;
float maxYstop = maxY;
float X, Y;
float length_1, length_2, length_3, length_4;
float curent_length_1, curent_length_2, curent_length_3, curent_length_4;
float change_in_length_1, change_in_length_2, change_in_length_3, change_in_length_4;

//scaling factors between pixels and steps
float scaleX = maxX / 320;
float scaleY = maxY / 240;
float spoolD = 10;//BagBag
float stepScale =  PI * spoolD / 200;

//starting lengths of cable
  // assuming in the centered position:
float start_length = sqrt(sq((maxX-minX)/2) + sq((maxY-minY)/2));
// @todo: if using center, don't need 4 different start lengths
float starting_length_1 = start_length;
float starting_length_2 = start_length;
float starting_length_3 = start_length;
float starting_length_4 = start_length;

//Variables to hold coordinates for end effector
float x, y, z = 1;

//Speed control variables
float maxSpeed;
float speedMult1, speedMult2, speedMult3, speedMult4;

//Endstop variables
bool endStop = false;
//Used to send endstop status back to host
// @todo convert to bool messages ASAP!! (floats are wasted here)
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


void catcherReturnHomeCb( const std_msgs::Bool& data);
ros::Subscriber<std_msgs::Bool> catcher_return_home("/catcher_return_home", &catcherReturnHomeCb);

#ifdef ros_everything

void twistMessageCb( const geometry_msgs::Twist& data);
void speedSetCb( const geometry_msgs::Point& data);

//Create ROS subscribers
ros::Subscriber<geometry_msgs::Twist> twist_subscriber("/cmd_vel_mux/input/teleop", &twistMessageCb);
ros::Subscriber<geometry_msgs::Point> motor_speed_subscriber("/motor_speed_set", &speedSetCb);
#endif
////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// END OF ROS PUBS & SUBS ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

void setup () {

  //Initialize node handler
  nh.initNode();
  //Initialize subscribers & publishers
  nh.subscribe(coordinate_subscriber);
  nh.subscribe(catcher_return_home);

#ifdef ros_everything
  nh.subscribe(twist_subscriber);
  nh.subscribe(motor_speed_subscriber);
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

////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// MAIN LOOP /////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
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

  endStopCheck();
  if ((ball_detected == 0)) {
    //step steppers once per main loop (can this be threaded?)
    // openDrain();
    /*
    stepper1.run();
    // openDrain();
    stepper2.run();
    // openDrain();
    stepper3.run();
    // openDrain();
    stepper4.run();
    // openDrain();

*/
      stepper1.runSpeedToPosition();
      // openDrain();
      stepper2.runSpeedToPosition();
      // openDrain();
      stepper3.runSpeedToPosition();
      // openDrain();
      stepper4.runSpeedToPosition ();
      // openDrain();

  


  }
  //Do ROS stuff once per main loop
  nh.spinOnce();
}


////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// CUSTOM FUNCTIONS /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
// @todo why unused? I don't know what system you've implemented any more ='(
// Given x & y directions from a camera, update the stepper velocities
void moveSteppers(int x, int y) {

  X = scaleX * camX;
  Y = scaleY * camY;

  //find new lengths
  lengthCal(X, Y);

  //find step counts
  stepper_counts_1 = length2Step(change_in_length_1);
  stepper_counts_2 = length2Step(change_in_length_2);
  stepper_counts_3 = length2Step(change_in_length_3);
  stepper_counts_4 = length2Step(change_in_length_4);

  // @todo Set stepper speeds here instead of positional movements
  //set motor positions
  stepper1.move(stepper_counts_1);
  stepper2.move(stepper_counts_2);
  stepper3.move(stepper_counts_3);
  stepper4.move(stepper_counts_4);
}

// Update the cord lengths & (x,y) position of the gripper
void updatePosition() {
  // get the current cord lengths based on the number of steps the motors have made
  curent_length_1 = (stepper1.currentPosition() * stepScale) + starting_length_1;
  curent_length_2 = (stepper2.currentPosition() * stepScale) + starting_length_2;
  curent_length_3 = (stepper3.currentPosition() * stepScale) + starting_length_3;
  curent_length_4 = (stepper4.currentPosition() * stepScale) + starting_length_4;

  // update the current (x,y) given the updated lengths
  current_x = distX / 2 + (sq(curent_length_3) - sq(curent_length_4)) / (2 * distX);
  current_y = distY / 2 + (sq(curent_length_4) - sq(curent_length_1)) / (2 * distY);
}

// Update cord length changes
void lengthCal(float dX, float dY) {
  updatePosition();

  // assuming dX & dY are positions, determine the lengths of cord required
  length_1 = sqrt(sq(maxX - (current_x + dX)) + sq(maxY - (current_y + dY)));
  length_2 = sqrt(sq(        current_x + dX)  + sq(maxY - (current_y + dY)));
  length_3 = sqrt(sq(        current_x + dX)  + sq(        current_y + dY));
  length_4 = sqrt(sq(maxX - (current_x + dX)) + sq(        current_y + dY));

  // @todo use this to determine velocity info
  //get dL(change in lengths)
  change_in_length_1 = length_1 - curent_length_1;
  change_in_length_2 = length_2 - curent_length_2;
  change_in_length_3 = length_3 - curent_length_3;
  change_in_length_4 = length_4 - curent_length_4;
}

// Convert a given length into a discrete number of motor steps
int length2Step(float dL) {
  int Step = round(200.0 * dL / (PI * spoolD));
  return Step;
}

// Determine if an endstop has been reached
void endStopCheck(void) {
  // Ensure the position is up to date
  updatePosition();

  // Determine if any of the endstop conditions have occured
  endStop1 = ((camX < 0) && (current_x <= minX)) ? true : false;    // Left
  endStop2 = ((camX > 0) && (current_x >= maxX)) ? true : false;    // Right
  endStop3 = ((camY < 0) && (current_y <= minY)) ? true : false;    // Bottom
  endStop4 = ((camY > 0) && (current_y >= maxY)) ? true : false;    // Top
  
  // Summarise
  endStop = (endStop1 || endStop2 || endStop3 || endStop4) ? true : false;
}


///////////////////////////////////////////////////////////////////////////////////////
// Message Callback Stuff//////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
//Message callback to populate x, y and z whenever a message is recieved on the coordinate_sender topic
// @todo throw up top with rest of global variables...
bool lFlag, rFlag, tFlag, bFlag;

// Find the new value of the maxSpeed variable
void findMax(float d1, float d2, float d3, float d4) {
  d1 = abs(d1);
  d2 = abs(d2);
  d3 = abs(d3);
  d4 = abs(d4);
  maxSpeed = max(d1, d2);
  maxSpeed = max(maxSpeed, d3);
  maxSpeed = max(maxSpeed, d4);
}

// Convert speeds to ratios of each other wrt the fastest
void setSpeedMult(float maxSpeed, float d1, float d2, float d3, float d4) {
  speedMult1 = d1 / maxSpeed;
  speedMult2 = d2 / maxSpeed;
  speedMult3 = d3 / maxSpeed;
  speedMult4 = d4 / maxSpeed;
}

// Set the speeds for each stepper motor
void setSpeed1(float speedM1, float speedM2, float speedM3, float speedM4) {
  int speed1, speed2, speed3, speed4;
  speed1 = round(stepperSpeed * speedM1);
  speed2 = round(stepperSpeed * speedM2);
  speed3 = round(stepperSpeed * speedM3);
  speed4 = round(stepperSpeed * speedM4);

  // @todo explain why set both setSpeed & setMaxSpeed or remove one of them.
  stepper1.setSpeed(speed1);
  stepper1.setMaxSpeed(speed1);
  stepper2.setSpeed(speed2);
  stepper2.setMaxSpeed(speed2);
  stepper3.setSpeed(speed3);
  stepper3.setMaxSpeed(speed3);
  stepper4.setSpeed(speed4);
  stepper4.setMaxSpeed(speed4);
}

// @todo needs a better explanation
//point comes in here
void messageCb( const geometry_msgs::Point& data) {

  stepper1.stop();
  stepper2.stop();
  stepper3.stop();
  stepper4.stop();

  //ENdstops are checked, if they're triggered then stop that axis from moving in that direction
  //Check whether the X or Y endstops have been hit. If they have, halt that axis
  if ((data.z > 160) && (current_x >= (maxX - (xBorder / 2)))) {
    camX = 0;
  } else if ((data.z < 160) && (current_x <= (minX + (xBorder / 2)))) {
    camX = 0;
  } else {
    //if endstop isn't triggered, then populate camX with a scaled (and zeroed) version of the point data
    camX = maxX * (data.z - 160) / 320;
  }
  //) ? camX = 0.5  : camX = maxX * (data.z - 160) / 320;


  if ((data.y > 120) && (current_y <= (minY + (yBorder / 2)))) {
    camY = 0;
  }
  else if ((data.y < 120) && (current_y >= (maxY - (yBorder / 2)))) {
    camY = 0;
  } else {
    camY = maxY * (120 - data.y) / 240;
  }



  // camY = maxY * (120 - data.y) / 2400;
  // camX = maxX * (data.z - 160) / 3200;


  //? camY = -0.5 : camY = maxY * (120 - data.y) / 240;
  //camY = maxY * (120 - data.y) / 240;
  ball_detected = 0;
#ifdef ros_everything
  quat_msg.x = camX;
  quat_msg.y = camY;
  quat_msg.z = current_x;
  quat_msg.w = current_y;
  cam_x_cam_y_current_x_current_y_pub.publish(&quat_msg);
#endif

  //scale (x,y) co-ordinates
  if ((ball_detected == 0)) { // && (!endStop)) {
    X = camX;
    Y = camY;

    //find new lengths
    lengthCal(X, Y);

    findMax(change_in_length_1, change_in_length_2, change_in_length_3, change_in_length_4);
    setSpeedMult(maxSpeed, change_in_length_1, change_in_length_2, change_in_length_3, change_in_length_4);
    setSpeed1(speedMult1, speedMult2, speedMult3, speedMult4);
  //  motorSetup();


    //find step counts
    stepper_counts_1 = length2Step(change_in_length_1);
    stepper_counts_2 = length2Step(change_in_length_2);
    stepper_counts_3 = length2Step(change_in_length_3);
    stepper_counts_4 = length2Step(change_in_length_4);



#ifdef ros_everything
    quat_msg.x = length_1;
    quat_msg.y = length_2;
    quat_msg.z = length_3;
    quat_msg.w = length_4;
    lengths_pub.publish(&quat_msg);
#endif
    //set motor positions
    stepper_counts_1 += stepper1.currentPosition();
    stepper_counts_2 += stepper2.currentPosition();
    stepper_counts_3 += stepper3.currentPosition();
    stepper_counts_4 += stepper4.currentPosition();



    stepper_counts_1 > step_last_1 ? direction_tracker_1 = 1 : direction_tracker_1 = 0;
    stepper_counts_2 > step_last_2 ? direction_tracker_2 = 1 : direction_tracker_2 = 0;
    stepper_counts_3 > step_last_3 ? direction_tracker_3 = 1 : direction_tracker_3 = 0;
    stepper_counts_4 > step_last_4 ? direction_tracker_4 = 1 : direction_tracker_4 = 0;

    //  stepper_counts_1 > step_last_1 ? digitalWrite(motor1_direction, HIGH) : digitalWrite(motor1_direction, LOW);
#ifdef ros_everything
    quat_msg.x = direction_tracker_1;
    quat_msg.y = direction_tracker_2;
    quat_msg.z = direction_tracker_3;
    quat_msg.w = direction_tracker_4;
    direction_tracker.publish(&quat_msg);
#endif
    step_last_1 = stepper_counts_1;
    step_last_2 = stepper_counts_2;
    step_last_3 = stepper_counts_3;
    step_last_4 = stepper_counts_4;

    stepper1.moveTo(stepper_counts_1);
    stepper2.moveTo(stepper_counts_2);
    stepper3.moveTo(stepper_counts_3);
    stepper4.moveTo(stepper_counts_4);
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
  stepper_counts_1 = length2Step(change_in_length_1);
  stepper_counts_2 = length2Step(change_in_length_2);
  stepper_counts_3 = length2Step(change_in_length_3);
  stepper_counts_4 = length2Step(change_in_length_4);

  //set motor positions
  stepper_counts_1 += stepper1.currentPosition();
  stepper_counts_2 += stepper2.currentPosition();
  stepper_counts_3 += stepper3.currentPosition();
  stepper_counts_4 += stepper4.currentPosition();
  stepper1.moveTo(stepper_counts_1);
  stepper2.moveTo(stepper_counts_2);
  stepper3.moveTo(stepper_counts_3);
  stepper4.moveTo(stepper_counts_4);

}

// @todo throw up top with rest of global variables...
int pulse_width = 200;
int motor_acceleration = 2000;

void speedSetCb( const geometry_msgs::Point & data) {
  pulse_width = data.x;
  stepperMaxSpeed = data.y;
  stepperSpeed = stepperMaxSpeed;
  motor_acceleration = data.z;
  motorSetup();
}





// @todo needs an explanation
// @todo all uses are commented out... delete if not necessary
// @todo appears to be something you would setup once and leave alone... if so put up top with the other setup stuff
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

// @todo needs an explanation
// @todo appears to be something you would setup once and leave alone... if so put up top with the other setup stuff
void motorSetup() {
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  stepper1.setMinPulseWidth(pulse_width);
  stepper1.setMaxSpeed(stepperMaxSpeed);
  stepper1.setSpeed(stepperSpeed);
  stepper1.setAcceleration(motor_acceleration);

  stepper2.setMinPulseWidth(pulse_width);
  stepper2.setMaxSpeed(stepperMaxSpeed);
  stepper2.setSpeed(stepperSpeed);
  stepper2.setAcceleration(motor_acceleration);

  stepper3.setMinPulseWidth(pulse_width);
  stepper3.setMaxSpeed(stepperMaxSpeed);
  stepper3.setSpeed(stepperSpeed);
  stepper3.setAcceleration(motor_acceleration);

  stepper4.setMinPulseWidth(pulse_width);
  stepper4.setMaxSpeed(stepperMaxSpeed);
  stepper4.setSpeed(stepperSpeed);
  stepper4.setAcceleration(motor_acceleration);
}

// Tells the system to return the gripper to the center position but has some weird ROS stuff attached
void catcherReturnHomeCb( const std_msgs::Bool & data) {
  if (!data.data) {
    returnHome();
  }
  else if (data.data) {
    stepper1.setCurrentPosition(0);
    stepper2.setCurrentPosition(0);
    stepper3.setCurrentPosition(0);
    stepper4.setCurrentPosition(0);
  }
}

// Tells the system to return the gripper to the center position
void returnHome() {
  stepper1.moveTo(0);
  stepper2.moveTo(0);
  stepper3.moveTo(0);
  stepper4.moveTo(0);
}
















