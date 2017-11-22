#include <PID_v1.h>
#include <AccelStepper.h>
//#include <PID_v1.h>
#include <ros.h>
#include <geometry_msgs/Point.h>
#include <ros/time.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>

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


float dataX = 160;
float dataY = 120;


//speeds
int pulse_width = 200;
int motor_acceleration = 2000;

//stepper speed variables
int stepperMaxSpeed = 600;
float stepperSpeed = 600;

//ROS publish timer stuff
unsigned long previousMillis = 0;  // last time update
long lInterval = 200;              // interval at which to publish ROS messages

float camX = 0;
float camY = 0;
float ball_detected = 1;
int steper_counts_1, steper_counts_2, steper_counts_3, steper_counts_4;


//Variables to hold information about the size of the frame, cable lengths etc...
//float maxX = 70.7, maxY = 70.7, current_x = 35, current_y = 35, X, Y;
int speed1, speed2, speed3, speed4;

float maxX = 71.5, maxY = 89.5;
float xBorder = 10;
float yBorder = 10;
float maxYstop = maxY;
float minX = 0;
float minY = 0;
float current_x = maxX / 2;
float current_y = maxY / 2;

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
//float stepScale =  PI * spoolD / 200;

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
bool endStopY = false;
bool endStopX = false;
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
///////////////////////////////// PID///////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////


float xKp = 1.4;
float xKi = 0.14;
float xKd = 0.01;//.5;

float yKp = 1.4;
float yKi = 0.14;
float yKd = 0.01;//.5;


double ySetpoint, yInput, yOutput;
double xSetpoint, xInput, xOutput;
double xout, zout, VL, VR;
double lError;

PID xPID(&xInput, &xOutput, &xSetpoint, xKp, xKi, xKd, DIRECT);
PID yPID(&yInput, &yOutput, &ySetpoint, yKp, yKi, yKd, DIRECT);

boolean PIDen = false;

////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// END OF PID ///////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// ROS PUBS & SUBS //////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////


//Create ROS nodeHandle
ros::NodeHandle nh;

//Subscribers
//Create Point and Quaternion messages (Point used to get ball location from host, quaternion used to publish motor/length data back to host)
geometry_msgs::Quaternion quat_msg;
geometry_msgs::Point point_msg;
geometry_msgs::Pose pid_all;


//Create publishers
ros::Publisher step_counts("step_counts", &quat_msg);
ros::Publisher lengths_pub("lengths_pub", &quat_msg);
ros::Publisher endstop_publisher("endstop_publisher", &quat_msg);
ros::Publisher direction_tracker("direction_tracker", &quat_msg);
ros::Publisher cam_x_cam_y_current_x_current_y_pub("cam_x_cam_y_current_x_current_y_pub", &quat_msg);
ros::Publisher motor_speed_publisher("motor_speed_publisher", &quat_msg);
ros::Publisher pid_publisher("pid_publisher", &pid_all);
//Create publisher to return gripper positions to hostPC. Create point_msg structure to contain this data
ros::Publisher current_pos("current_pos", &point_msg);
//Publishers
//ROS subscriber callback function prototypes
void messageCb( const geometry_msgs::Point& data);
ros::Subscriber<geometry_msgs::Point> coordinate_subscriber("coordinate_send_topic", &messageCb);


void catcherReturnHomeCb( const std_msgs::Bool& data);
ros::Subscriber<std_msgs::Bool> catcher_return_home("/catcher_return_home", &catcherReturnHomeCb);

void pid_all_cb(const geometry_msgs::Pose& pid_all);
void speedSetCb( const geometry_msgs::Point& data);

//Create ROS subscribers
ros::Subscriber<geometry_msgs::Pose> PID_tune_sub("PID_tuneAll", pid_all_cb);
ros::Subscriber<geometry_msgs::Point> motor_speed_subscriber("/motor_speed_set", &speedSetCb);
////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// END OF ROS PUBS & SUBS ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

void setup () {

  //Initialize node handler
  nh.initNode();
  //Initialize subscribers & publishers
  nh.subscribe(coordinate_subscriber);
  nh.subscribe(catcher_return_home);
  nh.subscribe(PID_tune_sub);

  nh.subscribe(motor_speed_subscriber);
  nh.advertise(step_counts);
  nh.advertise(lengths_pub);
  nh.advertise(endstop_publisher);
  nh.advertise(current_pos);
  nh.advertise(direction_tracker);
  nh.advertise(cam_x_cam_y_current_x_current_y_pub);
  nh.advertise(motor_speed_publisher);
  nh.advertise(pid_publisher);

  motorSetup();

  xPID.SetMode(AUTOMATIC);
  yPID.SetMode(AUTOMATIC);

  xPID.SetOutputLimits(0, 1300);
  yPID.SetOutputLimits(0, 1300);

  //messageCb( const geometry_msgs::Point& data)
  point_msg.x = 0;
  point_msg.y = 120;
  point_msg.z = 160;
  // messageCb(point_msg);
}


void loop() {

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

    quat_msg.x = speed1;
    quat_msg.y = speed2;
    quat_msg.z = speed3;
    quat_msg.w = speed4;

    motor_speed_publisher.publish(&quat_msg);
    pid_publisher.publish(&pid_all);

  }

  perform_calculations();


  if ((ball_detected == 0)) {
    //step steppers once per main loop (can this be threaded?)
    stepper1.runSpeed();
    stepper2.runSpeed();
    stepper3.runSpeed();
    stepper4.runSpeed();
  }
  //Do ROS stuff once per main loop
  nh.spinOnce();
}




void recalculate_gripper_position(void) {

  //get current lengths
  curent_length_1 = round((stepper1.currentPosition() * stepScale) + starting_length_1);
  curent_length_2 = round((stepper2.currentPosition() * stepScale) + starting_length_2);
  curent_length_3 = round((stepper3.currentPosition() * stepScale) + starting_length_3);
  curent_length_4 = round((stepper4.currentPosition() * stepScale) + starting_length_4);

  //get current (x,y) from lengths
  current_x = maxX / 2 + (sq(curent_length_3) - sq(curent_length_4)) / (2 * maxX);
  current_y = maxY / 2 + (sq(curent_length_4) - sq(curent_length_1)) / (2 * maxY);

}



void calculate_lengths(float & dX, float & dY) {

  recalculate_gripper_position();

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

float newposition, v_x, oldposition;
int newtime, oldtime;

float x_velocity_calculate(void) {
  // old_v_x = v_x;
  recalculate_gripper_position();
  newposition = current_x;
  newtime = millis();
  v_x = (newposition - oldposition) / (newtime - oldtime);
  oldposition = newposition;
  oldtime = newtime;
  return v_x;
  //    v_left = (v_left + old_v_left)/2;
}

float newposition_y, v_y, oldposition_y;
int newtime_y, oldtime_y;

float y_velocity_calculate(void) {
  // old_v_x = v_x;
  recalculate_gripper_position();
  newposition_y = current_y;
  newtime_y = millis();
  v_x = (newposition_y - oldposition_y) / (newtime_y - oldtime_y);
  oldposition_y = newposition_y;
  oldtime_y = newtime_y;
  return v_y;
  //    v_left = (v_left + old_v_left)/2;
}
//float32_msg.data = v_left;
//v_left_pub.publish( &float32_msg);

void perform_calculations(void) {
  camX = maxX * (dataX - 160) / 32;
  camY = maxY * (120 - dataY) / 24;

  xInput = x_velocity_calculate();
  xSetpoint = (dataX - 160);
  yInput = y_velocity_calculate();;
  ySetpoint = camY;
  ball_detected = 0;

  pid_all.position.x = xInput;
  pid_all.position.y = xSetpoint;

  pid_all.orientation.x = yInput;
  pid_all.orientation.y = ySetpoint;

  xPID.Compute();
  yPID.Compute();

  pid_all.position.z = xOutput;
  pid_all.orientation.x = yOutput;





  //scale (x,y) co-ordinates
  if ((ball_detected == 0)) {

    if (PIDen) {
      //find new lengths
      float xOut = xOutput;
      float yOut = yOutput;
      calculate_lengths(xOut, yOut);
    } else {
      //find new lengths
      calculate_lengths(camX, camY);
    }



    //calculate speeds and set them
    calculate_speeds(change_in_length_1, change_in_length_2, change_in_length_3, change_in_length_4);
    set_speeds();

    endStopCheck2();
  }
}


//point comes in here
//Message callback to populate x, y and z whenever a message is recieved on the coordinate_sender topic
void messageCb( const geometry_msgs::Point & data) {
  dataX = data.z;
  dataY = data.y;
  perform_calculations();

}


void speedSetCb( const geometry_msgs::Point & data) {
  pulse_width = data.x;
  stepperMaxSpeed = data.y;
  stepperSpeed = stepperMaxSpeed;
  motor_acceleration = data.z;
  motorSetup();
}



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


void catcherReturnHomeCb( const std_msgs::Bool & data) {
  if (!data.data) {
    returnHomeFlag = true;
  }
  else if (data.data) {
    stepper1.setCurrentPosition(0);
    stepper2.setCurrentPosition(0);
    stepper3.setCurrentPosition(0);
    stepper4.setCurrentPosition(0);
  }
}




void endStopCheck(void) {
  ((dataX < 160) && (current_x <= minX)) ? endstop1 = true : endstop1 = false;
  ((dataX > 160) && (current_x >= maxX)) ? endstop2 = true : endstop2 = false;
  ((dataY > 120) && (current_y <= minY)) ? endstop3 = true : endstop3 = false;
  ((dataY < 120) && (current_y >= maxY)) ? endstop4 = true : endstop4 = false;
}

bool endStopCheck2(void) {
  recalculate_gripper_position();

  if ((dataX > 160) && (current_x >= (maxX - (xBorder / 2)))) {
    endStopX = true;
    speedStop();
  } else if ((dataX < 160) && (current_x <= (minX + (xBorder / 2)))) {
    speedStop();
    endStopX = true;
  } else {
    endStopX = false;
  }


  if ((dataY < 120) && (current_y >= (maxY - (yBorder / 2)))) {
    speedStop();
    endStopY = true;
  }
  else if ((dataY > 120) && (current_y <= (minY + (yBorder / 2)))) {
    speedStop();
    endStopY = true;
  } else {
    endStopY = false;
  }

  if (endStopX || endStopY) {
    endStop = true;
  } else {
    endStop = false;
  }

  return endStop;

}


void speedStop(void) {
  stepper1.setSpeed(0);
  stepper1.setMaxSpeed(0);
  stepper2.setSpeed(0);
  stepper2.setMaxSpeed(0);
  stepper3.setSpeed(0);
  stepper3.setMaxSpeed(0);
  stepper4.setSpeed(0);
  stepper4.setMaxSpeed(0);
}


void calculate_speeds(float d1, float d2, float d3, float d4) {
  //take modulus of changes in length
  d1 = abs(d1);
  d2 = abs(d2);
  d3 = abs(d3);
  d4 = abs(d4);
  //find largest
  maxSpeed1 = max(d1, d2);
  maxSpeed1 = max(maxSpeed1, d3);
  maxSpeed1 = max(maxSpeed1, d4);
  //scale changes in length to largest (not the modulus versions)
  float speedM1 = change_in_length_1 / maxSpeed1;
  float speedM2 = change_in_length_2 / maxSpeed1;
  float speedM3 = change_in_length_3 / maxSpeed1;
  float speedM4 = change_in_length_4 / maxSpeed1;
  //apply global speed scaler
  speed1 = round(stepperSpeed * speedM1);
  speed2 = round(stepperSpeed * speedM2);
  speed3 = round(stepperSpeed * speedM3);
  speed4 = round(stepperSpeed * speedM4);
}

void set_speeds(void) {

  stepper1.setSpeed(speed1);
  stepper1.setMaxSpeed(speed1);
  stepper2.setSpeed(speed2);
  stepper2.setMaxSpeed(speed2);
  stepper3.setSpeed(speed3);
  stepper3.setMaxSpeed(speed3);
  stepper4.setSpeed(speed4);
  stepper4.setMaxSpeed(speed4);

}



void pid_all_cb(const geometry_msgs::Pose & pid_all) {
  xKp = pid_all.position.x;
  xKi = pid_all.position.y;
  xKd = pid_all.position.z;

  yKp = pid_all.orientation.x;
  yKi = pid_all.orientation.y;
  yKd = pid_all.orientation.z;

  PIDen = boolean(pid_all.orientation.w);

  xPID.SetTunings(xKp, xKi, xKd);
  yPID.SetTunings(yKp, yKi, yKd);
}

