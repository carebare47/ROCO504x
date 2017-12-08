#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from matplotlib import pyplot as plt
import time

global trails
x1 = 44.75
x2 = 35.75
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "X : %f   Y : %f", data.x,data.y)
    plt.clf()
    plt.ion()
    axes = plt.gca()
    axes.set_xlim([-10,100])
    axes.set_ylim([-10,80])
    axes.set_autoscale_on(False)
    x1=(data.x)
    x2=(data.y)
    plt.plot(x1, x2, '*')
   # plt.axis("equal")
    plt.draw()
    plt.pause(0.05)
    plt.show()


if __name__ == '__main__':
    rospy.init_node('plotter_node', anonymous=True)
    rospy.Subscriber("current_pos", Point, callback, queue_size = 1)
    rospy.spin()
    time.sleep(0.1)
    #plt.axis("equal")

