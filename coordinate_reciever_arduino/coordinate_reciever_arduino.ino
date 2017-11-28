  
#include <AccelStepper.h>
//#include <PID_v1.h>
#include <ros.h>
#include <geometry_msgs/Point.h>
#include <ros/time.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <math.h>

//Create accelstepper objects
int motor1_step = 3;
int motor1_direction = 2;

int motor2_step = 4;
int motor2_direction = 5;

int motor3_step = 6;
int motor3_direction = 7;

int motor4_step = 9;
int motor4_direction = 8;

float stepError1 = 0;
float stepError2 = 0;
float stepError3 = 0;
float stepError4 = 0;

float errorAccumulator1 = 0;
float errorAccumulator2 = 0;
float errorAccumulator3 = 0;
float errorAccumulator4 = 0;




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

float maxY = 71.5, maxX = 89.5;
float xBorder = 10;
float yBorder = 10;
float maxYstop = maxY;
float minX = 0;
float minY = 0;
float current_x = maxX / 2;
float current_y = maxY / 2;
float current_x_2 = maxX / 2;
float current_y_2 = maxY / 2;
float X, Y;

float length_1, length_2, length_3, length_4;
float current_length_1, current_length_2, current_length_3, current_length_4;
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
float starting_length_1 = 57.27674048;
float starting_length_2 = 57.27674048;
float starting_length_3 = 57.27674048;
float starting_length_4 = 57.27674048;

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
geometry_msgs::Quaternion quat_msg2;
std_msgs::Bool bool_msg;

#ifdef ros_everything
//Create publishers
ros::Publisher step_counts("step_counts", &quat_msg);
ros::Publisher lengths_pub("lengths_pub", &quat_msg);
ros::Publisher endstop_publisher("endstop_publisher", &quat_msg);
ros::Publisher direction_tracker("direction_tracker", &quat_msg);
ros::Publisher cam_x_cam_y_current_x_current_y_pub("cam_x_cam_y_current_x_current_y_pub", &quat_msg2);
ros::Publisher motor_speed_publisher("motor_speed_publisher", &quat_msg);
ros::Publisher loop_flag("loop_flag", &bool_msg);
ros::Publisher error_accumulator_pub("error_accumulator_pub", &quat_msg);
ros::Publisher flag_pub("flag_pub", &point_msg);

//Create publisher to return gripper positions to hostPC. Create point_msg structure to contain this data
ros::Publisher current_pos("current_pos", &point_msg);
ros::Publisher other_current_pos("other_current_pos", &point_msg);
#endif
//Publishers
//ROS subscriber callback function prototypes
void messageCb( const geometry_msgs::Point& data);
ros::Subscriber<geometry_msgs::Point> coordinate_subscriber("coordinate_send_topic", &messageCb);


void catcherReturnHomeCb( const std_msgs::Bool& data);
ros::Subscriber<std_msgs::Bool> catcher_return_home("/catcher_return_home", &catcherReturnHomeCb);

#ifdef ros_everything

void speedSetCb( const geometry_msgs::Point& data);

//Create ROS subscribers
ros::Subscriber<geometry_msgs::Point> motor_speed_subscriber("/motor_speed_set", &speedSetCb);
#endif
////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// END OF ROS PUBS & SUBS ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
bool calculation_flag = true;

void setup () {

  //Initialize node handler
  nh.initNode();
  //Initialize subscribers & publishers
  nh.subscribe(coordinate_subscriber);
  nh.subscribe(catcher_return_home);

#ifdef ros_everything
  nh.subscribe(motor_speed_subscriber);
  nh.advertise(step_counts);
  nh.advertise(lengths_pub);
  nh.advertise(endstop_publisher);
  nh.advertise(current_pos);
  nh.advertise(other_current_pos);
  nh.advertise(direction_tracker);
  nh.advertise(cam_x_cam_y_current_x_current_y_pub);
  nh.advertise(motor_speed_publisher);
  nh.advertise(loop_flag);
  nh.advertise(error_accumulator_pub);
  nh.advertise(flag_pub);
#endif
  motorSetup();

  //messageCb( const geometry_msgs::Point& data)
  point_msg.x = 0;
  point_msg.y = 120;
  point_msg.z = 160;
  // messageCb(point_msg);
}

bool boundFlag, deadFlag;

void loop() {
#ifdef ros_everything

  //Publish data back to ROS every lInterval milliseconds
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > lInterval) {
    previousMillis = currentMillis;

    recalculate_gripper_position();

    //Current gripper position
    point_msg.x = current_x;
    point_msg.y = current_y;
    point_msg.z = 0 ;
    current_pos.publish(&point_msg);

    point_msg.x = current_x_2;
    point_msg.y = current_y_2;
    other_current_pos.publish(&point_msg);

    //  quat_msg.x = camX;
    //quat_msg.y = camY;
    cam_x_cam_y_current_x_current_y_pub.publish(&quat_msg2);
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

    quat_msg.x = endStop;
    quat_msg.y = endstop2;
    quat_msg.z = endstop3;
    quat_msg.w = endstop4;

    endstop_publisher.publish(&quat_msg);

    quat_msg.x = speed1;
    quat_msg.y = speed2;
    quat_msg.z = speed3;
    quat_msg.w = speed4;

    motor_speed_publisher.publish(&quat_msg);

    bool_msg.data = calculation_flag;
    loop_flag.publish(&bool_msg);

    quat_msg.x = errorAccumulator1;
    quat_msg.y = errorAccumulator2;
    quat_msg.z = errorAccumulator3;
    quat_msg.w = errorAccumulator4;
    error_accumulator_pub.publish(&quat_msg);

    point_msg.x = boundFlag;
    point_msg.y = deadFlag;
    flag_pub.publish(&point_msg);

  }

#endif


  //int loop_timer = micros();


   perform_calculations();

  endStopCheck2();

  // endStopCal();
  // endStopCheck2();
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


      stepper1.runSpeedToPosition();
      // openDrain();
      stepper2.runSpeedToPosition();
      // openDrain();
      stepper3.runSpeedToPosition();
      // openDrain();
      stepper4.runSpeedToPosition ();
      // openDrain();
    */
    stepper1.run();
    // openDrain();
    stepper2.run();
    // openDrain();
    stepper3.run();
    // openDrain();
    stepper4.run();
    // openDrain();


  }

  endStopCheck2();


  if (returnHomeFlag) {
    restore_default_speeds();
    motorSetup();
    stepper1.moveTo(0);
    stepper2.moveTo(0);
    stepper3.moveTo(0);
    stepper4.moveTo(0);
    //while (((stepper1.currentPosition() != 0) && (stepper2.currentPosition() != 0) && (stepper3.currentPosition() != 0) && (stepper4.currentPosition() != 0) )) {
    while (!((stepper1.currentPosition() == 0) && (stepper2.currentPosition() == 0) && (stepper3.currentPosition() == 0) && (stepper4.currentPosition() == 0) )) {
      stepper1.runSpeedToPosition();
      stepper2.runSpeedToPosition();
      stepper3.runSpeedToPosition();
      stepper4.runSpeedToPosition();
      nh.spinOnce();
    }
    returnHomeFlag = false;
  }
  //Do ROS stuff once per main loop
  nh.spinOnce();
}









void recalculate_gripper_position(void) {
  //get current lengths
  current_length_1 = ((stepper1.currentPosition() * stepScale) + starting_length_1);
  current_length_2 = ((stepper2.currentPosition() * stepScale) + starting_length_2);
  current_length_3 = ((stepper3.currentPosition() * stepScale) + starting_length_3);
  current_length_4 = ((stepper4.currentPosition() * stepScale) + starting_length_4);

  //get current (x,y) from lengths
  current_x = maxX / 2 + (sq(current_length_3) - sq(current_length_4)) / (2 * maxX);
  current_y = maxY / 2 + (sq(current_length_4) - sq(current_length_2)) / (2 * maxY);


  current_x_2 = maxX / 2 + (sq(current_length_1) - sq(current_length_2)) / (2 * maxX);
  current_y_2 = maxY / 2 + (sq(current_length_3) - sq(current_length_1)) / (2 * maxY);
}


void calculate_lengths(float &dX, float &dY) {
  recalculate_gripper_position();

  //get new lengths
  length_1 = sqrt(sq(      current_x + dX)  + sq(maxY - (current_y + dY)));
  length_2 = sqrt(sq(maxX - (current_x + dX)) + sq(maxY - (current_y + dY)));
  length_3 = sqrt(sq(      current_x + dX)  + sq(        current_y + dY));
  length_4 = sqrt(sq(maxX - (current_x + dX)) + sq(        current_y + dY));
  length_1 = round(length_1);
  length_2 = round(length_2);
  length_3 = round(length_3);
  length_4 = round(length_4);

  //get dL(change in lengths)
  change_in_length_1 = length_1 - current_length_1;
  change_in_length_2 = length_2 - current_length_2;
  change_in_length_3 = length_3 - current_length_3;
  change_in_length_4 = length_4 - current_length_4;
}


float length2Step(float &dL) {

  float revs = dL / (PI * spoolD);
  float Step = revs * 200;

  return Step;
}

int hyst = 2;
int tolerance = 5;
void check_boundaries(void) {
  camX = checkAxisBounds(camX, current_x, (minX + 10), (maxX - 10));
  camY = checkAxisBounds(camY, current_y, (minY + 10), (maxY - 10));
  quat_msg2.x = camX;
  quat_msg2.y = camY;
  camX = checkAxisDeadBand(camX, tolerance);
  camY = checkAxisDeadBand(camY, tolerance);
  quat_msg2.z = camX;
  //quat_msg2.w = camY;
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

void endStopCal(void) {

  current_length_1 = ((stepper1.currentPosition()) * stepScale) + starting_length_1;
  current_length_2 = ((stepper2.currentPosition()) * stepScale) + starting_length_2;
  current_length_3 = ((stepper3.currentPosition()) * stepScale) + starting_length_3;
  current_length_4 = ((stepper4.currentPosition()) * stepScale) + starting_length_4;
  //get current (x,y) from lengths
  current_x = maxX / 2 + (sq(current_length_3) - sq(current_length_4)) / (2 * maxX);
  current_y = maxY / 2 + (sq(current_length_4) - sq(current_length_2)) / (2 * maxY);


  if (((camX < 0) && (current_x <= (minX + 15))) || ((camX > 0) && (current_x >= (maxX - 15)))
      || ((camY < 0) && (current_y <= (minY + 15)  )) || ((camY > 0) && (current_y >= (maxY - 15)))) {
    endStop = true;
    stepper1.move(0);
    stepper2.move(0);
    stepper3.move(0);
    stepper4.move(0);
  }
  else
    endStop = false;
}
bool lFlag, rFlag, tFlag, bFlag;
//Message callback to populate x, y and z whenever a message is recieved on the coordinate_sender topic
float dataX = 160;
float dataY = 120;
void perform_calculations(void) {

  camX = (dataX - 160);
  camY = (dataY - 120);

  recalculate_gripper_position();
  check_boundaries();

  ball_detected = 0;

  //scale (x,y) co-ordinates
  if ((ball_detected == 0)) { // && (!endStop)) {


    //find new lengths
    calculate_lengths(camX, camY);

    calculate_speeds(change_in_length_1, change_in_length_2, change_in_length_3, change_in_length_4);
    set_speeds();

    //find step counts
    stepError1 = length2Step(change_in_length_1);
    stepError2 = length2Step(change_in_length_2);
    stepError3 = length2Step(change_in_length_3);
    stepError4 = length2Step(change_in_length_4);


    steper_counts_1 = round(stepError1);
    steper_counts_2 = round(stepError2);
    steper_counts_3 = round(stepError3);
    steper_counts_4 = round(stepError4);

    errorAccumulator1 += stepError1;
    errorAccumulator2 += stepError2;
    errorAccumulator3 += stepError3;
    errorAccumulator4 += stepError4;

    //set motor positions
    steper_counts_1 += stepper1.currentPosition();
    steper_counts_2 += stepper2.currentPosition();
    steper_counts_3 += stepper3.currentPosition();
    steper_counts_4 += stepper4.currentPosition();


    stepper1.moveTo(steper_counts_1);
    stepper2.moveTo(steper_counts_2);
    stepper3.moveTo(steper_counts_3);
    stepper4.moveTo(steper_counts_4);
    endStopCheck2();
    calculation_flag = false;

  }
}


//point comes in here
void messageCb( const geometry_msgs::Point& data) {
  dataX = data.z;
  dataY = data.y;
  // perform_calculations();
  perform_calculations();


}



int pulse_width = 200;
int motor_acceleration = 2000;

//stepper speed variables
int stepperMaxSpeed = 600;
float stepperSpeed = 600;

void restore_default_speeds(void) {
  pulse_width = 200;
  motor_acceleration = 2000;

  //stepper speed variables
  stepperMaxSpeed = 600;
  stepperSpeed = 600;

}


void speedSetCb( const geometry_msgs::Point & data) {
  pulse_width = data.x;
  stepperMaxSpeed = data.y;
  stepperSpeed = stepperMaxSpeed;
  motor_acceleration = data.z;
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
  ((dataY < 120) && (current_y <= minY)) ? endstop3 = true : endstop3 = false;
  ((dataY > 120) && (current_y >= maxY)) ? endstop4 = true : endstop4 = false;
}
bool endStopX = false;
bool endStopY = false;

bool endStopCheck2(void) {
  if ((dataX > 160) && (current_x >= (maxX - (xBorder / 2)))) {
    endStopX = true;
    camX = 0;
  } else if ((dataX < 160) && (current_x <= (minX + (xBorder / 2)))) {
    camX = 0;
    endStopX = true;
  } else {
    endStopX = false;
  }


  if ((dataY > 120) && (current_y >= (maxY - (yBorder / 2)))) {
    camY = 0;
    endStopY = true;
  }
  else if ((dataY < 120) && (current_y <= (minY + (yBorder / 2)))) {
    camY = 0;
    endStopY = true;
  } else {
    endStopY = false;
  }

  if (endStopY || endStopX) {
    endStop = true;
  } else {
    endStop = false;
  }
  return endStop;

}


void speedStop(void) {
  /*
    stepper1.setSpeed(0);
    stepper1.setMaxSpeed(0);
    stepper2.setSpeed(0);
    stepper2.setMaxSpeed(0);
    stepper3.setSpeed(0);
    stepper3.setMaxSpeed(0);
    stepper4.setSpeed(0);
    stepper4.setMaxSpeed(0);

    stepper1.move(0);
    stepper2.move(0);
    stepper3.move(0);
    stepper4.move(0);
  */
}


float maxSpeedRamp = 0;
float beta = 0.9999;
void acceleration_scale(void) {
  if ((camX == 0) && (camY == 0)) {
    maxSpeedRamp = 0;
  } else {
    maxSpeedRamp = beta * maxSpeedRamp + (1 - beta) * stepperSpeed;
    quat_msg2.w = maxSpeedRamp;
  }
}


void calculate_speeds(float d1, float d2, float d3, float d4) {

  float diff1 = abs(d1);
  float diff2 = abs(d2);
  float diff3 = abs(d3);
  float diff4 = abs(d4);
  maxSpeed1 = max(diff1, diff2);
  maxSpeed1 = max(maxSpeed1, diff3);
  maxSpeed1 = max(maxSpeed1, diff4);
  speedMult1 = d1 / maxSpeed1;
  speedMult2 = d2 / maxSpeed1;
  speedMult3 = d3 / maxSpeed1;
  speedMult4 = d4 / maxSpeed1;

  acceleration_scale();
  speed1 = round(maxSpeedRamp * speedMult1);
  speed2 = round(maxSpeedRamp * speedMult2);
  speed3 = round(maxSpeedRamp * speedMult3);
  speed4 = round(maxSpeedRamp * speedMult4);
}




void set_speeds (void) {
  stepper1.setSpeed(speed1);
  stepper1.setMaxSpeed(speed1);
  stepper2.setSpeed(speed2);
  stepper2.setMaxSpeed(speed2);
  stepper3.setSpeed(speed3);
  stepper3.setMaxSpeed(speed3);
  stepper4.setSpeed(speed4);
  stepper4.setMaxSpeed(speed4);
}


