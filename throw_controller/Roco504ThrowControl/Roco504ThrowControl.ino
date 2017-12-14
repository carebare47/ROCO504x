#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>


int MotorControlPin = 9;    // LED connected to digital pin 9
int spoolCommand = 0;
int spoolMotorCommand = 128;

void spool_cb(const std_msgs::Int16 &spoolControl) {
  spoolCommand = spoolControl.data;
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16> ThrowSpool("throw_motor", &spool_cb);
void setup() {
    nh.initNode();
  //Initialize subscribers & publishers
  nh.subscribe(ThrowSpool);
  
}

void loop() {

  if (spoolCommand == 0) {
    spoolMotorCommand = 128;
    analogWrite(MotorControlPin, spoolMotorCommand);
  }

  else if (spoolCommand == 1) {
    spoolMotorCommand = 0;
    analogWrite(MotorControlPin, spoolMotorCommand);
  }

  else if (spoolCommand == 2) {
    spoolMotorCommand = 255;
    analogWrite(MotorControlPin, spoolMotorCommand);
  }

  nh.spinOnce();

}
