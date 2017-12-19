# ROCO504x

This is the git repository for the ROCO504 catching and throwing robot project.

Written for use with ROS Kinetic.
ROS Kinetic installation instructions: http://wiki.ros.org/kinetic/Installation/Ubuntu

Once ROS Kinetic is installed, set up your catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Install the dynamixel motor drivers for ROS: http://wiki.ros.org/dynamixel_motor

```
sudo apt-get install ros-$ROS_DISTRO-dynamixel-motor --y
```
or 
```
sudo apt-get install ros-kinetic-dynamixel-motor --y
```
Install webcam drivers and QV4L2:
```
sudo apt-get install v4l-* qv4l2
```
Clone this repository to catkin_ws/src, then from catkin_ws, type `catkin_make`

Copy ROCO504x/USBserialRules/99-usb-serial.rules to /etc/udev/rules.d and apply rules to assign mappings to the USBtoDynamixel adapters
```
sudo cp ~/catkin_ws/src/ROCO504x/USBserialRules/99-usb-serial.rules /etc/udev/rules.d
sudo udevadm trigger
```

The following step will vary depending on which throw motor you wish to use. An example is provided for an arduino UNO on rosserial connected to a geared micro DC motor via a pololu JRK motor controller. 

Install/setup rosserial: https://github.com/ros-drivers/rosserial

Program the arduino with code found in ROCO504x/throw_controller/ROCO504ThrowControl/ROCO504ThrowControl.ino


Then, in four terminals, start the following programmes:
```
roslaunch catcher_frame_controller controller_managerx2.launch
rosrun kinematic_controller kinematic_controller2 
rosrun throw_controller throw_ctroller_manual 
rosrun hough_ball_tracker hough_ball_tracker 
```
If you wish to visualise where the robot thinks the gripper is, then run `rosrun catch_bot_plot 3_catch_bot_plot.py`


Here's a video of it working: https://youtu.be/0yMIV64DRBU

And here are the CAD files created for this project: https://github.com/carebare47/ROCO504xCAD


TODO:
Discuss / add license information
