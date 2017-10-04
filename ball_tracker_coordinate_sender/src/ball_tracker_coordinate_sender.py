#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Point

def coordinate_sender():
	pub = rospy.Publisher('coordinate_send_topic', Point, queue_size=10)
	rospy.init_node('coordinate_sender', anonymous=True)
	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		rospy.loginfo("populating point message")
		msg.linear.x = 1
		msg.linear.y = 2
		msg.linear.z = 3
		pub.publish(msg)
		rate.sleep()


if __name__ == '__main__':
    try:
        coordinate_sender()
    except rospy.ROSInterruptException:
        pass
