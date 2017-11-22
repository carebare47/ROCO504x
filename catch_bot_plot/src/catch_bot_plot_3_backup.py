#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from matplotlib import pyplot as plt
import time

plt.ioff()

def callback(data):
    plt.cla() 
    plt.plot(data.x, data.y, '*')

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
    plt.axis([0, 360, 0, 240])
    ax = plt.gca()
    ax.set_autoscale_on(False)
    plt.plot(120, 160, 'h')
    plt.axis("equal")
    plt.draw()
    plt.pause(0.00000000001)   
    time.sleep(3)
    plt.show()
