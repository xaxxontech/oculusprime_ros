#!/usr/bin/env python


import rospy, tf
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path



def odomCallback(data):
	odomx = data.pose.pose.position.x
	odomy = data.pose.pose.position.y
	quaternion = ( data.pose.pose.orientation.x, data.pose.pose.orientation.y,
	data.pose.pose.orientation.z, data.pose.pose.orientation.w )
	odomth = tf.transformations.euler_from_quaternion(quaternion)[2]
	print str(odomx)+", "+str(odomy)+", "+str(odomth)

# MAIN

rospy.init_node('base_controller', anonymous=False)
rospy.Subscriber("odom", Odometry, odomCallback)


while not rospy.is_shutdown():
	pass


