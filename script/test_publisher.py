#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

def test_publisher():
	rospy.init_node('test_publisher')
	pub = rospy.Publisher("/obj_coord", Point, queue_size = 1)
	rate = rospy.Rate(10)
	msg = Point()

	msg.x = 10
	msg.y = 40
	msg.z = 0
	for i in range(5):
		pub.publish(msg)
		rate.sleep()

	msg.x = 240
	msg.y = 100
	msg.z = 0
	for i in range(5):
		pub.publish(msg)
		rate.sleep()

#	msg.x = 127
#	msg.y = 69
#	msg.z = 0
#	for i in range(5):
#		pub.publish(msg)
#		rate.sleep()

if __name__ == "__main__":
	try:
		test_publisher()
	except rospy.ROSInterruptException:
		pass
