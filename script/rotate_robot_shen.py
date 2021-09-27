#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

def Rotate_robot():
	rospy.init_node('Rotate_robot',anonymous = True)
	rate = rospy.Rate(10)
	rospy.Subscriber("/obj_coord",Point,callback,queue_size=10)



def callback(data):
	#print(data)
	global pub
	Width = 410
	#eps = 10
	vel_msg = Twist()
	vel_msg.angular.z = 0

	if data.x < 0.45*Width:
		vel_msg.angular.z = 0.15

	if data.x > 0.55*Width:
		vel_msg.angular.z = -0.15
	pub.publish(vel_msg)
    #rospy.loginfo('publish: ' + str(msg.x))



if __name__ == "__main__":
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
	while True:
		try:
			Rotate_robot()
			rospy.spin()
		except rospy.ROSInterruptException:
			rospy.spin()