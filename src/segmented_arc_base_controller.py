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
/move_base_simple/goal
"""

import rospy, tf
import socketclient
from nav_msgs.msg import Odometry
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

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
minturn = 0.19
lastpath = 0
goalpose = False

def pathCallback(data):
	global targetx, targety, targetth, followpath, lastpath
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
	global goalth 
	quaternion = ( data.pose.orientation.x, data.pose.orientation.y,
	data.pose.orientation.z, data.pose.orientation.w )
	goalth = tf.transformations.euler_from_quaternion(quaternion)[2]
	

def move(ox, oy, oth, tx, ty, tth, gth):
	# (only move if over min threshold)
	
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
	else:
		th = tth
		
	if goalpose:
		th = gth
		distance = 0
		
	""" scenarios:
	current = -170, target = 0 >> move = 170
	current = 170, target = -170 >> move = 20 (=360+t-c)
	current = 50, target = -90 >> move = -140  (=t-c)
	current = 0, target = 180 >> move = -180 or 180 (=t-c)
	current = -10, target = 170 >> move = -180 or 180  (=t-c)
	current = -10, target = 175 >> move = -175 (=-360+t-c)
	current = 50, target == 20 >> move = -30  (=t-c)
	current = -50, target = -10 >> move = 40  (=t-c)
	current = 1, target = -1
	curent = 185, target = 0
	"""
	
	dth = th - oth
	if dth > math.pi:
		dth = -math.pi*2 + dth
	elif dth < -math.pi:
		dth = math.pi*2 + dth
		
	# set minimums	
	if distance > 0 and distance < 0.05:
		distance = 0.05
	# minimum rotate is currently 8 degrees! (fix in firmware...) OR turn slower
	if dth < minturn*0.4 and dth > -minturn*0.4:
		dth = 0
	elif dth >= minturn*0.4 and dth < minturn:
		dth = minturn
	elif dth <= -minturn*0.4 and dth > -minturn:
		dth = -minturn
	
	# print "move: "+str(distance)+", "+str(dth)+", "+str(th)
	
	if dth > 0:
		socketclient.sendString("left "+str(int(math.degrees(dth))) )
		socketclient.waitForReplySearch("<state> direction stop")
	elif dth < 0:
		socketclient.sendString("right "+str(abs(int(math.degrees(dth)))) )
		socketclient.waitForReplySearch("<state> direction stop")

	if distance > 0:
		socketclient.sendString("forward "+str(distance))
		socketclient.waitForReplySearch("<state> direction stop")
	
def cleanup():
	socketclient.sendString("odometrystop")
	socketclient.sendString("state stopbetweenmoves false")
	socketclient.sendString("move stop")


# MAIN

rospy.init_node('base_controller', anonymous=False)
rospy.Subscriber("move_base/TrajectoryPlannerROS/local_plan", Path, pathCallback)
rospy.Subscriber("odom", Odometry, odomCallback)
rospy.Subscriber("move_base_simple/goal", PoseStamped, goalCallback)
rospy.on_shutdown(cleanup)

while not rospy.is_shutdown():
	t = rospy.get_time()
	if t >= nextmove:
		move(odomx, odomy, odomth, targetx, targety, targetth, goalth)
		nextmove = t+listentime
		followpath = False
	if t - lastpath > 3 and not t - lastpath > 15:
		goalpose = True
		
cleanup()
