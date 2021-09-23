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

	def callback(self, data):
		rospy.loginfo('received x: ' + str(data.x) + ', y: ' + str(data.y))
		self.objX = data.x
		self.vel_msg.angular.z = self.transform(self.objX)
		self.pub.publish(self.vel_msg)

	def transform(self, x): #quick and dirty fall-through logic tree
		if (x < (self.WIDTH/2 - self.deadZ)):
			return 0.25
		if (x > (self.WIDTH/2 + self.deadZ)):
			return -0.25 #FIXME need to confirm polarity
		return 0

def rotate_robot():
	rospy.init_node('rotate_robot')
	c = controller()
	rospy.Subscriber("/obj_coord", Point, c.callback)

	rospy.spin()

if __name__ == "__main__":
	rotate_robot()
