# ROCO504x

So far, this is the catkin src workspace for the ROCO504 project.

Written for use with ROS Kinetic.
ROS Kinetic installation instructions: http://wiki.ros.org/kinetic/Installation/Ubuntu

Once ROS Kinetic is installed, set up your catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Install the dynamixel motor drivers for ROS: http://wiki.ros.org/dynamixel_motor

Clone this repository to catkin_ws/src
Then from catkin_ws, type `catkin_make`

Copy ROCO504x/USBserialRules/99-usb-serial.rules to /etc/udev/rules.d to assign mappings to the USBtoDynamixel adapters
```
sudo cp ~/catkin_ws/src/ROCO504x/USBserialRules/99-usb-serial.rules /etc/udev/rules.d
```

The following step will vary depending on which throw motor you wish to use. An example is provided for an arduino UNO on rosserial connected to a geared micro DC motor via a pololu JRK motor controller. 

Install/setup rosserial: https://github.com/ros-drivers/rosserial

Program the arduino with code found in ROCO504X/throw_controller/ROCO504ThrowControl/ROCO504ThrowControl.ino


Then, in four terminals, start the following programmes:
```
roslaunch catcher_frame_controller controller_managerx2.launch
rosrun kinematic_controller kinematic_controller2 
rosrun throw_controller throw_ctroller_manual 
rosrun hough_ball_tracker hough_ball_tracker 
```


TODO:

Link to CAD files
Discuss / add license information
