#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

class controller:
	def __init__(self):
		self.WIDTH = 410 #FIXME please update this with width of frame
		self.deadZ = 10 #deadzone of 10 pixels
		self.objX = self.WIDTH/2 #initialized to midpoint of frame to prevent premature movement
		self.vel_msg = Twist()
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
		self.safety_delay = rospy.Duration(2) #secs
		self.safety_stopTime = rospy.get_rostime()

	def sub_callback(self, data):
		rospy.loginfo('received x: ' + str(data.x) + ', y: ' + str(data.y))
		self.objX = data.x
		self.vel_msg.angular.z = self.transform(self.objX)
		self.pub.publish(self.vel_msg)
		self.safety_stopTime = rospy.get_rostime() + self.safety_delay

	def safety_stop(self):
		if (self.vel_msg.angular.z != 0):
			self.vel_msg.angular.z = 0
			self.pub.publish(self.vel_msg)

	def transform(self, x): #quick and dirty fall-through logic tree
		if (x < (self.WIDTH/2 - self.deadZ)):
			return 0.25
		if (x > (self.WIDTH/2 + self.deadZ)):
			return -0.25
		return 0

def rotate_robot():
	rospy.init_node('rotate_robot')
	c = controller()
	rospy.Subscriber("/obj_coord", Point, c.sub_callback)
	while not rospy.is_shutdown():
		if (rospy.get_rostime() > c.safety_stopTime):
			c.safety_stop()
if __name__ == "__main__":
	rotate_robot()
