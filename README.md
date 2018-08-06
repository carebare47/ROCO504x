# ROCO504x - CatchBot

Video of the catch-bot working: https://youtu.be/lfRBLpXKWZI

## Installation and setup
### ROS
Written for use with ROS Kinetic.
ROS Kinetic installation instructions: http://wiki.ros.org/kinetic/Installation/Ubuntu

Once ROS Kinetic is installed, set up your catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```
If kinetic is the only version of ROS you are running, then run the following to automatically source your new terminals (and your current one):
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Dynamixel
Install the dynamixel motor drivers for ROS: http://wiki.ros.org/dynamixel_motor

```
sudo apt-get install ros-$ROS_DISTRO-dynamixel-motor --y
```
or 
```
sudo apt-get install ros-kinetic-dynamixel-motor --y
```
### Camera
Install webcam drivers, QV4L2, usb support and uvcdynctrl (used to allow the tracking program to setup the camera feed):
```
sudo apt-get install v4l-* qv4l2 libusb-dev uvcdynctrl
```
### CatchBot
Clone this repository to catkin_ws/src and build:
```
cd ~/catkin_ws/src/
git clone https://github.com/carebare47/ROCO504x
cd ..
catkin_make
```

#### udev rules setup
Copy ROCO504x/USBserialRules/99-usb-serial.rules to /etc/udev/rules.d and apply rules to assign mappings to the USBtoDynamixel adapters (you will have to edit this file if you don't use the physical adapters that are currently connected to our robot, as the rules file uses unique serial numbers for each device):
```
sudo cp ~/catkin_ws/src/ROCO504x/USBserialRules/99-usb-serial.rules /etc/udev/rules.d
sudo udevadm trigger
```

### Throw motor setup
The following step will vary depending on which throw motor you wish to use. An example is provided for an arduino UNO on rosserial connected to a geared micro DC motor via a pololu JRK motor controller. 

Install/setup rosserial: https://github.com/ros-drivers/rosserial

Program the arduino with code found in ROCO504x/throw_controller/ROCO504ThrowControl/ROCO504ThrowControl.ino


## Running the robot
Then, in four terminals, start the following programmes:
```
roslaunch catcher_frame_controller controller_managerx2.launch
rosrun kinematic_controller kinematic_controller2 
rosrun throw_controller throw_controller_manual 
rosrun hough_ball_tracker hough_ball_tracker 
```

## Visualising (for debug)
If you wish to visualise where the robot thinks the gripper is, then run `rosrun catch_bot_plot 3_catch_bot_plot.py`

## CAD files
Here are the CAD files created for this project: https://github.com/carebare47/ROCO504xCAD


## Vectorized Flowchart (Only describes tracking algorithm)
![alt text](https://raw.githubusercontent.com/carebare47/ROCO504x/master/ROCO504-finished-flowchart.png)
