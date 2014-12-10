#!/usr/bin/env python

"""
listen to /move_base/TrajectoryPlannerROS/local_plan
use rotate and movedistance commands to get there, repeat
subscribe to odo so moves relative to current pos
assumes robot is stopped before running this

rosmsg show nav_msgs/Path
rosmsg show nav_msgs/Odometry
rosmsg show geometry_msgs/PoseStamped << is in map frame!!!

consider polling telnet and broadcasting odom from here, to only update odom between moves
/move_base/status  3 = goal reached, 1= accepted (read last in list)
rosmsg show actionlib_msgs/GoalStatusArray

adding initial pose to goal pose:
1st test, initial poseth -90deg: solution would be just ot add initial pose to goal pose
	gth = gth + ith
	180 = 180 + (-90)
	if dth > math.pi:
		dth = -math.pi*2 + dth
	elif dth < -math.pi:
		dth = math.pi*2 + dth
^^ works but not if odom gets whacked - need to use odom/map tf and ADD to goal pose diff
"""

import rospy, tf
import oculusprimesocket
from nav_msgs.msg import Odometry
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped #, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray

listentime = 1.5 # constant, seconds
nextmove = 0
odomx = 0
odomy = 0
odomth = 0
targetx = 0
targety = 0
targetth = 0
followpath = False
goalth = 0 
minturn = math.radians(6) # 0.21 minimum for pwm 255
lastpath = 0
goalpose = False
goalseek = False
linearspeed = 150
secondspermeter = 3.2 #float
turnspeed = 100
secondspertwopi = 3.8
initth = 0
#initgoalth = 0
tfth = 0

def pathCallback(data):
	global targetx, targety, targetth, followpath, lastpath, goalpose
	lastpath = rospy.get_time()
	goalpose = False
	followpath = True
	p = data.poses[len(data.poses)-1] # get latest pose
	targetx = p.pose.position.x
	targety = p.pose.position.y
	quaternion = ( p.pose.orientation.x, p.pose.orientation.y,
	p.pose.orientation.z, p.pose.orientation.w )
	targetth = tf.transformations.euler_from_quaternion(quaternion)[2]

def odomCallback(data):
	global odomx, odomy, odomth
	odomx = data.pose.pose.position.x
	odomy = data.pose.pose.position.y
	quaternion = ( data.pose.pose.orientation.x, data.pose.pose.orientation.y,
	data.pose.pose.orientation.z, data.pose.pose.orientation.w )
	odomth = tf.transformations.euler_from_quaternion(quaternion)[2]
	
def goalCallback(data):
	global goalth, followpath, lastpath, goalpose, odomx, odomy, odomth, targetx, targety, targetth
	goalpose = False
	followpath = True
	lastpath = 0
	targetx = odomx
	targety = odomy
	
	if goalpose:
		targetth = goalth # - tfth
	else:
		targetth = odomth  # + tfth	 
		
	# if targetth > math.pi:
		# targetth = -math.pi*2 + targetth
	# elif targetth < -math.pi:
		# targetth = math.pi*2 + targetth
		
	quaternion = ( data.pose.orientation.x, data.pose.orientation.y,
	data.pose.orientation.z, data.pose.orientation.w )
	goalth = tf.transformations.euler_from_quaternion(quaternion)[2]
	# goalth -= tfth
	# if goalth > math.pi:
		# goalth = -math.pi*2 + goalth
	# elif goalth < -math.pi:
		# goalth = math.pi*2 + goalth
	
def goalStatusCallback(data):
	global goalseek
	goalseek = False
	if len(data.status_list) == 0:
		return
	status = data.status_list[len(data.status_list)-1] # get latest status
	if status.status == 1:
		goalseek = True
		
# def initialposeCallback(data):
	# global initth
	# quaternion = ( data.pose.pose.orientation.x, data.pose.pose.orientation.y,
	# data.pose.pose.orientation.z, data.pose.pose.orientation.w )
	# initth = tf.transformations.euler_from_quaternion(quaternion)[2]

# def amclposeCallback(data):
	# global goalth, initgoalth
	# quaternion = ( data.pose.pose.orientation.x, data.pose.pose.orientation.y,
	# data.pose.pose.orientation.z, data.pose.pose.orientation.w )
	# amclth = tf.transformations.euler_from_quaternion(quaternion)[2]
	# goalth = initgoalth - amclth
	# if goalth > math.pi:
		# goalth = -math.pi*2 + goalth
	# elif goalth < -math.pi:
		# goalth = math.pi*2 + goalth

def move(ox, oy, oth, tx, ty, tth, gth):
	global followpath, goalpose, tfth

	# print "odom: "+str(ox)+", "+str(oy)+", "+str(oth)
	# print "target: "+str(tx)+", "+str(ty)+", "+str(tth)
	
	distance = 0
	if followpath:
		dx = tx - ox
		dy = ty - oy	
		distance = math.sqrt( pow(dx,2) + pow(dy,2) )
	
	if distance > 0:
		th = math.acos(dx/distance)
		if dy <0:
			th = -th
	elif goalpose:
		th = gth - tfth
	else:
		th = tth

	#if goalpose:
		#th = gth
		#distance = 0
	
	dth = th - oth
	if dth > math.pi:
		dth = -math.pi*2 + dth
	elif dth < -math.pi:
		dth = math.pi*2 + dth
		
	# force minimums	
	if distance > 0 and distance < 0.05:
		distance = 0.05

	# supposed to reduce zig zagging
	if dth < minturn*0.3 and dth > -minturn*0.3:
		dth = 0
	elif dth >= minturn*0.3 and dth < minturn:
		dth = minturn
	elif dth <= -minturn*0.3 and dth > -minturn:
		dth = -minturn


	if dth > 0:
		oculusprimesocket.sendString("speed "+str(turnspeed) )
		oculusprimesocket.sendString("move left")
		rospy.sleep(dth/(2.0*math.pi) * secondspertwopi)
		oculusprimesocket.sendString("move stop")
		oculusprimesocket.waitForReplySearch("<state> direction stop")
	elif dth < 0:
		oculusprimesocket.sendString("speed "+str(turnspeed) )
		oculusprimesocket.sendString("move right")
		rospy.sleep(-dth/(2.0*math.pi) * secondspertwopi)
		oculusprimesocket.sendString("move stop")
		oculusprimesocket.waitForReplySearch("<state> direction stop")

	if distance > 0:
		oculusprimesocket.sendString("speed "+str(linearspeed) )
		oculusprimesocket.sendString("move forward")
		rospy.sleep(distance*secondspermeter)
		oculusprimesocket.sendString("move stop")
		oculusprimesocket.waitForReplySearch("<state> direction stop")
	
def cleanup():
	oculusprimesocket.sendString("odometrystop")
	oculusprimesocket.sendString("state stopbetweenmoves false")
	oculusprimesocket.sendString("move stop")


# MAIN

rospy.init_node('base_controller', anonymous=False)
rospy.Subscriber("move_base/TrajectoryPlannerROS/local_plan", Path, pathCallback)
rospy.Subscriber("odom", Odometry, odomCallback)
rospy.Subscriber("move_base_simple/goal", PoseStamped, goalCallback)
rospy.Subscriber("move_base/status", GoalStatusArray, goalStatusCallback)
# rospy.Subscriber("initialpose", PoseWithCovarianceStamped, initialposeCallback)
# rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, amclposeCallback)
rospy.on_shutdown(cleanup)
listener = tf.TransformListener()

while not rospy.is_shutdown():
	t = rospy.get_time()
	
	if t >= nextmove and goalseek:
		move(odomx, odomy, odomth, targetx, targety, targetth, goalth)
		nextmove = t + listentime
		# nextmove = rospy.get_time()+listentime
		followpath = False
	
	if t - lastpath > 3:
		goalpose = True
	
	try:
		(trans,rot) = listener.lookupTransform('/map', '/odom', rospy.Time(0))
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		continue		
	quaternion = (rot[0], rot[1], rot[2], rot[3])
	tfth = tf.transformations.euler_from_quaternion(quaternion)[2]
		
cleanup()
