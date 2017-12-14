# ROCO504x

So far, this is the catkin src workspace for the ROCO504 project.

Written for use with ROS Indigo.
ROS Indigo installation instructions: http://wiki.ros.org/indigo/Installation/Ubuntu

Once ROS Indigo is installed, set up your catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
Clone this repository to catkin_ws/src
Then from catkin_ws, type `catkin_make`

Copy ROCO504x/USBserialRules/99-usb-serial.rules to /etc/udev/rules.d to assign mappings to the USBtoDynamixel adapters
```
sudo cp ~/catkin_ws/src/ROCO504x/USBserialRules/99-usb-serial.rules /etc/udev/rules.d
```
Then, in four terminals, start the following programmes:
```
roslaunch catcher_frame_controller controller_managerx2.launch
rosrun kinematic_controller kinematic_controller2 
rosrun throw_controller throw_ctroller_manual 
rosrun hough_ball_tracker houghall_tracker 
```


TODO:


Discuss / add license information
