#include "kinematic_controller.hpp"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinematic_controller");
  ros::NodeHandle n;
  ros::ServiceClient client1 = n.serviceClient<dynamixel_controllers::SetSpeed>("/motor1_controller/set_speed");
  ros::ServiceClient client2 = n.serviceClient<dynamixel_controllers::SetSpeed>("/motor2_controller/set_speed");
  ros::ServiceClient client3 = n.serviceClient<dynamixel_controllers::SetSpeed>("/motor3_controller/set_speed");
  ros::ServiceClient client4 = n.serviceClient<dynamixel_controllers::SetSpeed>("/motor4_controller/set_speed");

  ros::Subscriber cameraSub = n.subscribe("coordinate_send_topic", 1, cameraDataCallback);
  ros::Subscriber motor1Sub = n.subscribe("/motor1_controller/state", 1, m1Callback);
  ros::Subscriber motor2Sub = n.subscribe("/motor2_controller/state", 1, m2Callback);
  ros::Subscriber motor3Sub = n.subscribe("/motor3_controller/state", 1, m3Callback);
  ros::Subscriber motor4Sub = n.subscribe("/motor4_controller/state", 1, m4Callback);

  ros::Publisher motor1Pub = n.advertise<std_msgs::Float64>("/motor1_controller/command", 1);
  ros::Publisher motor2Pub = n.advertise<std_msgs::Float64>("/motor2_controller/command", 1);
  ros::Publisher motor3Pub = n.advertise<std_msgs::Float64>("/motor3_controller/command", 1);
  ros::Publisher motor4Pub = n.advertise<std_msgs::Float64>("/motor4_controller/command", 1);
  ros::Publisher bound_flag_pub = n.advertise<std_msgs::Bool>("/bound_flag", 1);

  ros::Publisher motor1simPub = n.advertise<dynamixel_msgs::JointState>("/motor1_controller/state", 1);
  ros::Publisher motor2simPub = n.advertise<dynamixel_msgs::JointState>("/motor2_controller/state", 1);
  ros::Publisher motor3simPub = n.advertise<dynamixel_msgs::JointState>("/motor3_controller/state", 1);
  ros::Publisher motor4simPub = n.advertise<dynamixel_msgs::JointState>("/motor4_controller/state", 1);

  ros::Publisher motorLengthPub = n.advertise<geometry_msgs::Quaternion>("motorLengths", 1);
  ros::Publisher motorPositionsPub = n.advertise<geometry_msgs::Quaternion>("motorPositions", 1);
  ros::Publisher PositionPub = n.advertise<geometry_msgs::Point>("current_pos", 1);
  speedStruct motorSpeeds;

  ros::Rate loop_rate(250);
  bool initFlag = true;


	while (ros::ok()){
    // goFlag = false;
		if (goFlag == true){
			if (ball_detected == 1.0){
				dataX = 0.0;
				dataY = 0.0;
			}
			newLengths = findNewLengths(dataX, dataY);
			motorSpeeds = calculate_speeds(newLengths);
			bound_flag_obj.data = boundFlag;
			bound_flag_pub.publish(bound_flag_obj);
			XY.x = newLengths.x;
			XY.y = newLengths.y;
			lengths.x = newLengths.nl1;
			lengths.y = newLengths.nl2;
			lengths.z = newLengths.nl3;
			lengths.w = newLengths.nl4;
			positions.x = m1Position;
			positions.y = m2Position;
			positions.z = m3Position;
			positions.w = m4Position;
      PositionPub.publish(XY);
      motorLengthPub.publish(lengths);
			motorPositionsPub.publish(positions);

			//setSpeedSrv(client1,client2,client3,client4,srv,motorSpeeds);

			m1NewPos.data = motorLengthToPosition(newLengths.l1) + m1Position - tension;
			m2NewPos.data = motorLengthToPosition(newLengths.l2) + m2Position - tension;
			m3NewPos.data = motorLengthToPosition(newLengths.l3) + m3Position - tension;
			m4NewPos.data = motorLengthToPosition(newLengths.l4) + m4Position - tension;
      motor1sim.current_pos = m1NewPos.data;
      motor2sim.current_pos = m2NewPos.data;
      motor3sim.current_pos = m3NewPos.data;
      motor4sim.current_pos = m4NewPos.data;

			motor1Pub.publish(m1NewPos);
			motor2Pub.publish(m2NewPos);
			motor3Pub.publish(m3NewPos);
			motor4Pub.publish(m4NewPos);

      motor1simPub.publish(motor1sim);
      motor2simPub.publish(motor2sim);
      motor3simPub.publish(motor3sim);
      motor4simPub.publish(motor4sim);
			initFlag = false;
			goFlag = false;
		}
		else if (initFlag == true){

			float starting_gripper_offset = motorLengthToPosition(half_gripper_diagonal);
			m1NewPos.data = PI - starting_gripper_offset - tension;//correct
			m2NewPos.data = PI - starting_gripper_offset - tension;//correct
			m3NewPos.data = PI - starting_gripper_offset - tension;//correct
			m4NewPos.data = PI - starting_gripper_offset - tension;//correct?
      ROS_INFO("offset = %f",starting_gripper_offset);
			motorSpeeds.s1 = 10.0;
			motorSpeeds.s2 = 10.0;
			motorSpeeds.s3 = 10.0;
			motorSpeeds.s4 = 10.0;

			//setSpeedSrv(client1,client2,client3,client4,srv,motorSpeeds);

			motor1Pub.publish(m1NewPos);
			motor2Pub.publish(m2NewPos);
			motor3Pub.publish(m3NewPos);
			motor4Pub.publish(m4NewPos);

      motor1sim.current_pos = m1NewPos.data;
      motor2sim.current_pos = m2NewPos.data;
      motor3sim.current_pos = m3NewPos.data;
      motor4sim.current_pos = m4NewPos.data;

      motor1simPub.publish(motor1sim);
      motor2simPub.publish(motor2sim);
      motor3simPub.publish(motor3sim);
      motor4simPub.publish(motor4sim);


			ros::spinOnce();
			loop_rate.sleep();
		}
    else if (initFlag == false){

			// newLengths = findNewLengths(0, 0);
			// motorSpeeds = calculate_speeds(newLengths);
			// XY.x = newLengths.x;
			// XY.y = newLengths.y;
			// lengths.x = newLengths.nl1;
			// lengths.y = newLengths.nl2;
			// lengths.z = newLengths.nl3;
			// lengths.w = newLengths.nl4;
			// positions.x = m1Position;
			// positions.y = m2Position;
			// positions.z = m3Position;
			// positions.w = m4Position;
			// PositionPub.publish(XY);
			// motorLengthPub.publish(lengths);
			// motorPositionsPub.publish(positions);

		}

		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}


/*
        y+ 
   2---------1
    (+)----(.)
x+  ----------  x-
    (.)----(+)
   4---------3
     -------
      -----
       -5-
        y-

gripper_starting_motor_offset 
0.9375
+PI = 4.07909
-PI = 2.20409
*/
