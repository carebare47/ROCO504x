#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from matplotlib import pyplot as plt
import time

plt.ioff()
global trails
def callback(data):
    if data.z == 0:
        plt.cla() 
    plt.plot(data.x, data.y, '*')
    plt.axis("equal")
    axes.set_xlim([0,320])
    axes.set_ylim([0,240])
    plt.draw()
    plt.pause(0.00000000001)   
    plt.show()

 
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('plotter_node', anonymous=True)
    rospy.Subscriber("current_pos", Point, callback)
    plt.show(block=True)
    # spin() simply keeps python from exiting until this node is stopped
#    rospy.spin()

if __name__ == '__main__':
    listener()
    plt.plot(160, 120, 'h')
    plt.axis("equal")
    axes = plt.gca()
    axes.set_xlim([0,320])
    axes.set_ylim([0,240])
    plt.draw()
    plt.pause(0.00000000001)   
    time.sleep(3)
    plt.show()
