#!/usr/bin/env python

import rospy, tf, math
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray

odomx = 0
odomy = 0
odomth = 0
targetx = 0
targety = 0
targetth = 0
goalth = 0
goalstatus = 0
initposeth = 0
amclth = 0
tfth = 0

def pathCallback(data):
	global targetx, targety, targetth
	p = data.poses[len(data.poses)-1] # get latest pose
	targetx = p.pose.position.x
	targety = p.pose.position.y
	quaternion = ( p.pose.orientation.x, p.pose.orientation.y,
	p.pose.orientation.z, p.pose.orientation.w )
	targetth = tf.transformations.euler_from_quaternion(quaternion)[2]
	dump()

def odomCallback(data):
	global odomx, odomy, odomth
	odomx = data.pose.pose.position.x
	odomy = data.pose.pose.position.y
	quaternion = ( data.pose.pose.orientation.x, data.pose.pose.orientation.y,
	data.pose.pose.orientation.z, data.pose.pose.orientation.w )
	odomth = tf.transformations.euler_from_quaternion(quaternion)[2]
	dump()
	
def goalCallback(data):
	global goalth
	quaternion = ( data.pose.orientation.x, data.pose.orientation.y,
	data.pose.orientation.z, data.pose.orientation.w )
	goalth = tf.transformations.euler_from_quaternion(quaternion)[2]
	dump()
	
def goalStatusCallback(data):
	global goalstatus
	if len(data.status_list) == 0:
		return
	status = data.status_list[len(data.status_list)-1] # get latest status
	goalstatus = status.status
	dump()

def initialposeCallback(data):
	global initposeth
	quaternion = ( data.pose.pose.orientation.x, data.pose.pose.orientation.y,
	data.pose.pose.orientation.z, data.pose.pose.orientation.w )
	initposeth = tf.transformations.euler_from_quaternion(quaternion)[2]
	dump()
	
def amclposeCallback(data):
	global amclth
	quaternion = ( data.pose.pose.orientation.x, data.pose.pose.orientation.y,
	data.pose.pose.orientation.z, data.pose.pose.orientation.w )
	amclth = tf.transformations.euler_from_quaternion(quaternion)[2]
	
def dump():
	global odomth, targetth, goalth, goalstatus, amclth, initposeth, tfth
	print "oth: "+str(odomth)+", tth: "+str(targetth)+", gth: "+str(goalth)+", gs: " \
		+str(goalstatus)+", ith: "+str(initposeth)+", ath: "+str(amclth)+", tfth: "+str(tfth)

# MAIN

rospy.init_node('blob', anonymous=False)
rospy.Subscriber("move_base/TrajectoryPlannerROS/local_plan", Path, pathCallback)
rospy.Subscriber("odom", Odometry, odomCallback)
rospy.Subscriber("move_base_simple/goal", PoseStamped, goalCallback)
rospy.Subscriber("move_base/status", GoalStatusArray, goalStatusCallback)
rospy.Subscriber("initialpose", PoseWithCovarianceStamped, initialposeCallback)
rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, amclposeCallback)
listener = tf.TransformListener()

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
	try:
		(trans,rot) = listener.lookupTransform('/map', '/odom', rospy.Time(0))
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		continue
		
	#tfth = 4 * math.atan2(trans[1], trans[0])
	quaternion = (rot[0], rot[1], rot[2], rot[3])
	tfth = tf.transformations.euler_from_quaternion(quaternion)[2]

	rate.sleep()
	


