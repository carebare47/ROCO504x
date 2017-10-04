#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Point
import cv2

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


def startCamera():
	rospy.loginfo("Create video capture pointer")
	cap = cv2.VideoCapture(1)

	rospy.loginfo("Setting resolution to 320x240")
	cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 320)
	cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 240)

	rospy.loginfo("Setting frame rate")
	cap.set(cv2.cv.CV_CAP_PROP_FPS, 250)

	


if __name__ == '__main__':
	startVideo()
	try:
		coordinate_sender()
	except rospy.ROSInterruptException:
        	pass
