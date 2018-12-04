#!/usr/bin/env python

"""
on any new /initialpose, do full rotation, then delay (to hone in amcl)

requires dwa_base_controller, global path updated continuously as bot moves

"""


import rospy, tf
import oculusprimesocket
from nav_msgs.msg import Odometry
import math, re
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal

listentime = 0.6 # allows odom + amcl to catch up
nextmove = 0
odomx = 0
odomy = 0
odomth = 0
gptargetx = 0	
gptargety = 0
gptargetth = 0
lptargetx = 0	
lptargety = 0
lptargetth = 0
followpath = False
pathid = None
goalx = 0
goaly = 0
goalth = 0 
initialturn = False
waitonaboutface = 0
minturn = math.radians(8) # (was 6) -- 0.21 minimum for pwm 255
minlinear = 0.08 # was 0.05
maxlinear = 0.5
# maxarclinear = 0.75
lastpath = 0  # refers to localpath
goalpose = False
goalseek = False

# TODO: get these two values from java
meterspersec = 0.33 # linear speed  
radianspersec = 1.496 # 1.496 = 0.0857degrees per ms

dpmthreshold = 1.2 # maximum to allow arcturn, radians per meter
tfx = 0
tfy = 0
tfth = 0
globalpathposenum = 20  
listener = None


def pathCallback(data): # local path
	global goalpose, lastpath, followpath
	global lptargetx, lptargety  
	
	lastpath = rospy.get_time()
	goalpose = False
	
	p = data.poses[len(data.poses)-1] # get last pose in path
	lptargetx = p.pose.position.x
	lptargety = p.pose.position.y
	quaternion = ( p.pose.orientation.x, p.pose.orientation.y,
	p.pose.orientation.z, p.pose.orientation.w )
	lptargetth = tf.transformations.euler_from_quaternion(quaternion)[2]
	
def globalPathCallback(data):
	global gptargetx, gptargety, gptargetth, followpath, pathid
	
	n = len(data.poses)
	if n < 5:
		return
		
	if n-1 < globalpathposenum:
		p = data.poses[n-1] 
	else:
		p = data.poses[globalpathposenum]
	
	gptargetx = p.pose.position.x
	gptargety = p.pose.position.y
	quaternion = ( p.pose.orientation.x, p.pose.orientation.y,
	p.pose.orientation.z, p.pose.orientation.w )
	gptargetth = tf.transformations.euler_from_quaternion(quaternion)[2]
	
	pathid = data.header.seq
	followpath = True


def odomCallback(data):
	global odomx, odomy, odomth
	odomx = data.pose.pose.position.x
	odomy = data.pose.pose.position.y
	quaternion = ( data.pose.pose.orientation.x, data.pose.pose.orientation.y,
	data.pose.pose.orientation.z, data.pose.pose.orientation.w )
	odomth = tf.transformations.euler_from_quaternion(quaternion)[2]
	

def intialPoseCallback(data):
	if data.pose.pose.position.x == 0 and data.pose.pose.position.y == 0:
		return
	# do full rotation on pose estimate, to hone-in amcl (if not docked)
	rospy.sleep(0.5) # let amcl settle
	# oculusprimesocket.clearIncoming()  
	# oculusprimesocket.sendString("right 360")
	# oculusprimesocket.waitForReplySearch("<state> direction stop")
	
def goalCallback(d):
	global goalx, goaly, goalth, goalpose, lastpath, initialturn, followpath, nextmove

	# to prevent immediately rotating wrongly towards new goal direction 
	lastpath = rospy.get_time()
	goalpose = False

	# set goal angle
	data = d.goal.target_pose
	goalx = data.pose.position.x
	goaly = data.pose.position.y
	quaternion = ( data.pose.orientation.x, data.pose.orientation.y,
	data.pose.orientation.z, data.pose.orientation.w )
	goalth = tf.transformations.euler_from_quaternion(quaternion)[2]
	initialturn = True
	followpath = False
	nextmove = lastpath + 2 # sometimes globalpath still points at previoius goal
	
def goalStatusCallback(data):
	global goalseek
	goalseek = False
	if len(data.status_list) == 0:
		return
	status = data.status_list[len(data.status_list)-1] # get latest status
	if status.status == 1:
		goalseek = True
				
def arcmove(ox, oy, oth, gpx, gpy, gpth, gth, lpx, lpy, lpth):
	
	global initialturn, waitonaboutface, nextmove	
	
	# global path targets
	gpdx = gpx - ox
	gpdy = gpy - oy	
	gpdistance = math.sqrt( pow(gpdx,2) + pow(gpdy,2) )
	if not gpdistance == 0:
		gpth = math.acos(gpdx/gpdistance)
		if gpdy <0:
			gpth = -gpth
		
	# local path targets	
	lpdx = lpx - ox
	lpdy = lpy - oy	
	lpdistance = math.sqrt( pow(lpdx,2) + pow(lpdy,2) )
	if not lpdistance == 0:
		lpth = math.acos(lpdx/lpdistance)
		if lpdy <0:
			lpth = -lpth
			
	# find arclength if any, and turn radians, depending on scenario
	arclength = 0
	goalrotate = False	
	if followpath and not ox==gpx and not oy==gpy and not initialturn: # normal arc move
			
		if abs(lpth-gpth) > dpmthreshold: # 90 degrees local path disparity, use global instead
			dth = gpth
			distance = gpdistance/2 # prevent oscillation, better than distance = 0
			# distance = 0
			# print("using global path")
		else:
			dth = lpth
			distance = lpdistance

		dth = dth - oth
		if dth > math.pi:
			dth = -math.pi*2 + dth
		elif dth < -math.pi:
			dth = math.pi*2 + dth

		radius = (distance/2)/math.sin(dth/2)
		arclength = radius * dth # *should* work out to always be > 0
		if not arclength == 0:
			if abs(dth/arclength) > dpmthreshold:
				arclength = 0
				# print ("high dpm")

	elif goalpose:  # final goal rotate move
		try:
			(trans,rot) = listener.lookupTransform('/map', '/odom', rospy.Time(0))
			quaternion = (rot[0], rot[1], rot[2], rot[3])
			dth = (gth - tf.transformations.euler_from_quaternion(quaternion)[2]) - oth
			goalrotate = True
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			dth = lpth

	else: # initial turn move (always?)  
		dth = gpth - oth # point to global path

	# determine angle delta for move TODO: cleanup
	if dth > math.pi:
		dth = -math.pi*2 + dth
	elif dth < -math.pi:
		dth = math.pi*2 + dth
		
	# if making large turns-in-place, wait, let planner stabilize
	if abs(dth) > 1.57 and not goalrotate and not initialturn and waitonaboutface < 1: 
		if goalDistance() > 0.9: # skip if close to goal
			# oculusprimesocket.clearIncoming()
			# oculusprimesocket.sendString("forward 0.25")
			# oculusprimesocket.waitForReplySearch("<state> direction stop")
			waitonaboutface += 1 # only do this once
			rospy.sleep(1.5)
			nextmove = rospy.get_time() + listentime
			return
	waitonaboutface = 0

	initialturn = False

	if arclength > 0: # arcmove
		if arclength < minlinear:
			arclength = minlinear

		oculusprimesocket.sendString("arcmove " + str(arclength) + " " + str(int(math.degrees(dth))) ) 
		rospy.sleep(arclength/meterspersec)
		nextmove = rospy.get_time()
		return

	
	# rotate only

	# force minimum
	if dth > 0 and dth < minturn:
		dth = minturn
	elif dth < 0 and dth > -minturn:
		dth = -minturn
	
	oculusprimesocket.clearIncoming()

	oculusprimesocket.sendString("move stop")
	# oculusprimesocket.waitForReplySearch("<state> direction stop")
	rospy.sleep(0.5)

	# if dth > 0:
		# oculusprimesocket.sendString("left " + str(int(math.degrees(dth))) ) 
		# oculusprimesocket.waitForReplySearch("<state> direction stop")
	# elif dth < 0:
		# oculusprimesocket.sendString("right " +str(int(math.degrees(-dth))) )
		# oculusprimesocket.waitForReplySearch("<state> direction stop")
	
	if not dth == 0:
		oculusprimesocket.sendString("rotate " + str(int(math.degrees(dth))) ) 
		oculusprimesocket.waitForReplySearch("<state> odomrotating false")
		
	nextmove = rospy.get_time() + listentime
	

def move(ox, oy, oth, tx, ty, tth, gth):
	global followpath, initialturn, waitonaboutface, nextmove

	currentpathid = pathid

	# determine xy deltas for move
	distance = 0
	if followpath:
		dx = tx - ox
		dy = ty - oy	
		distance = math.sqrt( pow(dx,2) + pow(dy,2) )
	
	goalrotate = False
	if distance > 0:
		th = math.acos(dx/distance)
		if dy <0:
			th = -th
			
	elif goalpose:
		try:
			(trans,rot) = listener.lookupTransform('/map', '/odom', rospy.Time(0))
			quaternion = (rot[0], rot[1], rot[2], rot[3])
			th = gth - tf.transformations.euler_from_quaternion(quaternion)[2]
			goalrotate = True
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			th = tth		
			
		# th = gth - tfth

	else: # does this ever get called? just use th = math.acos(dx/distance), same?
		th = tth
	
	# determine angle delta for move
	dth = th - oth
	if dth > math.pi:
		dth = -math.pi*2 + dth
	elif dth < -math.pi:
		dth = math.pi*2 + dth
		
	# force minimums	
	if distance > 0 and distance < minlinear:
		distance = minlinear
		
	if distance > maxlinear:
		distance = maxlinear

	# supposed to reduce zig zagging  (was 0.3)
	if dth < minturn*0.5 and dth > -minturn*0.5:
		dth = 0
	elif dth >= minturn*0.5 and dth < minturn:
		dth = minturn
	elif dth <= -minturn*0.5 and dth > -minturn:
		dth = -minturn

	oculusprimesocket.clearIncoming()

	# if making large turns-in-place, wait, let planner stabilize
	if abs(dth) > 1.57 and not goalrotate and not initialturn and waitonaboutface < 1: 
		if goalDistance() > 0.9: # skip if close to goal
			# oculusprimesocket.clearIncoming()
			# oculusprimesocket.sendString("forward 0.25")
			# oculusprimesocket.waitForReplySearch("<state> direction stop")
			waitonaboutface += 1 # only do this once
			rospy.sleep(1.5)
			nextmove = rospy.get_time() + listentime
			return
	waitonaboutface = 0
	
	initialturn = False

	if not pathid == currentpathid:			
		nextmove = rospy.get_time() + listentime
		return

	# if dth > 0:
		# oculusprimesocket.sendString("left " + str(int(math.degrees(dth))) ) 
		# oculusprimesocket.waitForReplySearch("<state> direction stop")
	# elif dth < 0:
		# oculusprimesocket.sendString("right " +str(int(math.degrees(-dth))) )
		# oculusprimesocket.waitForReplySearch("<state> direction stop")
	
	if not dth == 0:	
		oculusprimesocket.sendString("rotate " + str(int(math.degrees(dth))) ) 
		oculusprimesocket.waitForReplySearch("<state> odomrotating false")

	if distance > 0:
		oculusprimesocket.sendString("forward "+str(distance))
		rospy.sleep(distance/meterspersec)
	
	nextmove = rospy.get_time() + listentime


def goalDistance(): 
	
	try:
		(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		return 99

	gdx = goalx - trans[0]
	gdy = goaly - trans[1]
	distance  = math.sqrt( pow(gdx,2) + pow(gdy,2) )
	# print ("goaldistance: "+str(distance)+", tfx: "+str(tfx)+", tfy: "+str(tfy))
	return distance
	
			
def cleanup():
	oculusprimesocket.sendString("log arcmove_globalpath_follower.py disconnecting")   


# MAIN

# rospy.init_node('dwa_base_controller', anonymous=False)
rospy.init_node('arcmove_globalpath_follower', anonymous=False)
listener = tf.TransformListener()
oculusprimesocket.connect()
rospy.on_shutdown(cleanup)

rospy.Subscriber("odom", Odometry, odomCallback)
rospy.Subscriber("move_base/DWAPlannerROS/local_plan", Path, pathCallback)
rospy.Subscriber("move_base/goal", MoveBaseActionGoal, goalCallback)
rospy.Subscriber("move_base/status", GoalStatusArray, goalStatusCallback)
rospy.Subscriber("move_base/DWAPlannerROS/global_plan", Path, globalPathCallback)
rospy.Subscriber("initialpose", PoseWithCovarianceStamped, intialPoseCallback)

oculusprimesocket.sendString("log arcmove_globalpath_follower.py connected") 
oculusprimesocket.sendString("readsetting usearcmoves") 
s = oculusprimesocket.waitForReplySearch("setting usearcmoves")
if re.search("true", s):
	oculusprimesocket.sendString("state rosarcmove true") 
else:
	oculusprimesocket.sendString("state rosarcmove false") 

oculusprimesocket.sendString("state rotatetolerance 5")   # set rotate target angle tolerance (default is 2, unnecessarily tight)


while not rospy.is_shutdown():
	t = rospy.get_time()
	
	if t >= nextmove:
		if goalseek and (followpath or goalpose):
			
			oculusprimesocket.sendString("state rosarcmove")  
			s = oculusprimesocket.waitForReplySearch("<state> rosarcmove")
			if re.search("true", s):
				rosarcmove = True
			else:
				rosarcmove = False
				
			if rosarcmove and goalDistance() > 0.9:
				arcmove(odomx, odomy, odomth, gptargetx, gptargety, gptargetth, goalth, lptargetx, lptargety, lptargetth) # blocking
			else:
				# print ("using move() global path")
				move(odomx, odomy, odomth, gptargetx, gptargety, gptargetth, goalth) # blocking
				
			followpath = False
	
	if t - lastpath > 3 and goalseek:
		if goalDistance() <= 0.9:
			goalpose = True
		else:
			rospy.sleep(0.5)
		# goalpose = True
	
	rospy.sleep(0.01)
	
