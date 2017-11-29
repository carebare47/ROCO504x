#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void cameraDataCallback(const geometry_msgs::Point::ConstPtr& data){
perform
}

void perform_calculations(void) {

  camX = (dataX - 160);
  camY = (dataY - 120);

  recalculate_gripper_position();
  check_boundaries();



  ball_detected = 0;

  //scale (x,y) co-ordinates
  if ((ball_detected == 0)) {


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
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::Subscriber cameraSub = n.subscribe("coordinate_send_topic", 1000, cameraDataCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
