#!/usr/bin/env python

"""
on any new /initialpose, do full rotation, then delay
"""


import rospy, tf
import oculusprimesocket
from nav_msgs.msg import Odometry
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal

listentime = 1.1 # magic constant, seconds  0.8 with sim_time = 1.0
nextmove = 0
odomx = 0
odomy = 0
odomth = 0
targetx = 0	
targety = 0
targetth = 0
followpath = False
goalx = 0
goaly = 0
goalth = 0 
minturn = math.radians(6) # 0.21 minimum for pwm 255
lastpath = 0
goalpose = False
goalseek = False
linearspeed = 150
secondspermeter = 3.2 #float
turnspeed = 100
secondspertwopi = 4.2
initth = 0
#initgoalth = 0
tfth = 0
gbpathx = 0
gbpathy = 0
initturn = False
# lastodomupdate = 0

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
	
def globalPathCallback(data):
	global gbpathx, gbpathy
	n = len(data.poses)
	if n > 0:
		p = data.poses[int(n*0.1)] #[len(data.poses)-1] # choose pose 10% along path
		gbpathx = p.pose.position.x
		gbpathy = p.pose.position.y

def odomCallback(data):
	global odomx, odomy, odomth #, lastodomupdate
	odomx = data.pose.pose.position.x
	odomy = data.pose.pose.position.y
	quaternion = ( data.pose.pose.orientation.x, data.pose.pose.orientation.y,
	data.pose.pose.orientation.z, data.pose.pose.orientation.w )
	odomth = tf.transformations.euler_from_quaternion(quaternion)[2]
	
	# if rospy.get_time() - lastodomupdate > 0.5:
		# oculusprimesocket.sendString("state rosodom "+str(odomx)+"_"+str(odomy)+"_"+str(odomth))
		# lastodomupdate = rospy.get_time()

def intialPoseCallback(data):
	# do full rotation on pose estimate, to hone-in amcl
	global initturn
	initturn = True
	rospy.sleep(0.5) # let amcl settle
	oculusprimesocket.sendString("speed "+str(turnspeed) )
	oculusprimesocket.sendString("move right")
	rospy.sleep(secondspertwopi) # full rotation
	oculusprimesocket.sendString("move stop")
	oculusprimesocket.waitForReplySearch("<state> direction stop")
	rospy.sleep(1)
	initturn = False
	
def goalCallback(d):
	global goalth, goalx, goaly, followpath, lastpath, goalpose
	global odomx, odomy, odomth
	global gbpathx, gbpathy, initturn
	
	while initturn == True: # wait if another initturn already in progress
		pass
	
	# set goal angle
	data = d.goal.target_pose
	# data =d
	goalx = data.pose.position.x
	goaly = data.pose.position.y
	quaternion = ( data.pose.orientation.x, data.pose.orientation.y,
	data.pose.orientation.z, data.pose.orientation.w )
	goalth = tf.transformations.euler_from_quaternion(quaternion)[2]

	# turn towards global path before doing anything
	goalpose = False	
	initturn = True
	
	# wait for global path
	gbpathx = None
	t = rospy.get_time()
	lastpath = t
	while gbpathx == None and rospy.get_time() < t + 5.0: 
		pass
	
	if not gbpathx == None:
		dx = gbpathx - odomx
		dy = gbpathy - odomy	
		distance = math.sqrt( pow(dx,2) + pow(dy,2) )
		if distance > 0:
			gbth = math.acos(dx/distance)
			if dy <0:
				gbth = -gbth
			move(0, 0, odomth, 0, 0, gbth, gbth)  # turn only 
			rospy.sleep(0.5) # let amcl settle
			
	initturn = False

	lastpath = rospy.get_time()
	followpath = False
	
def goalStatusCallback(data):
	global goalseek
	goalseek = False
	if len(data.status_list) == 0:
		return
	status = data.status_list[len(data.status_list)-1] # get latest status
	if status.status == 1:
		goalseek = True

def move(ox, oy, oth, tx, ty, tth, gth):
	global followpath, goalpose, tfth, nextmove
	global goalx, goaly, odomx, odomy, odomth

	# print "odom: "+str(ox)+", "+str(oy)+", "+str(oth)
	# print "target: "+str(tx)+", "+str(ty)+", "+str(tth)
	
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
		th = gth - tfth
		goalrotate = True
	else:
		th = tth
	
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
	
	if goalrotate:
		rospy.sleep(1) 
	
	# if not dth == 0 and distance == 0 and not goalrotate:
		# nextmove = rospy.get_time() + 0.5
	
def cleanup():
	oculusprimesocket.sendString("odometrystop")
	oculusprimesocket.sendString("state stopbetweenmoves false")
	oculusprimesocket.sendString("move stop")


# MAIN

rospy.init_node('dwa_base_controller', anonymous=False)
oculusprimesocket.connect()
rospy.Subscriber("move_base/DWAPlannerROS/local_plan", Path, pathCallback)
rospy.Subscriber("odom", Odometry, odomCallback)
rospy.Subscriber("move_base/goal", MoveBaseActionGoal, goalCallback)
rospy.Subscriber("move_base/status", GoalStatusArray, goalStatusCallback)
rospy.Subscriber("move_base/DWAPlannerROS/global_plan", Path, globalPathCallback)
rospy.Subscriber("initialpose", PoseWithCovarianceStamped, intialPoseCallback)
rospy.on_shutdown(cleanup)
listener = tf.TransformListener()


while not rospy.is_shutdown():
	t = rospy.get_time()
	
	if t >= nextmove:
		nextmove = t + listentime
		if goalseek and not initturn:
			move(odomx, odomy, odomth, targetx, targety, targetth, goalth)
			followpath = False
	
	if t - lastpath > 3:
		goalpose = True
	
	if int(t- lastpath) > 10 and goalseek: # recovery behavior, rotate
		oculusprimesocket.sendString("speed "+str(turnspeed) )
		oculusprimesocket.sendString("move right")
		rospy.sleep(1)
# 
	# if t- lastpath > 20 and goalseek: # failure, exit
		# rospy.loginfo("oculusprime base contoller exit")
		# break
		
	try:
		(trans,rot) = listener.lookupTransform('/map', '/odom', rospy.Time(0))
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		continue		
	quaternion = (rot[0], rot[1], rot[2], rot[3])
	tfth = tf.transformations.euler_from_quaternion(quaternion)[2]
		
cleanup()
