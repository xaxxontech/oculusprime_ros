#!/usr/bin/env python

"""
on any new /initialpose, do full rotation, then delay (to hone in amcl)

follow something ~15th pose in global path for all moves (about 0.3m away?)
    -maximum path length seems to be about 35*5 (45*5 max) for 2-3 meter path
    -(longer if more turns -- go for 15th or 20th pose, or max if less, should be OK)

ignore local path, except for determining if at goal or not
	if no recent local path, must be at goal: followpath = False, goalpose = true

requires dwa_base_controller, global path updated continuously as bot moves

"""


import rospy, tf
import oculusprimesocket
from nav_msgs.msg import Odometry
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal

listentime = 0.5 # allows odom + amcl to catch up
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
lastpath = 0  # refers to localpath
goalpose = False
goalseek = False
linearspeed = 150
secondspermeter = 3.2 # calibration, automate? (do in java, faster)
turnspeed = 100
secondspertwopi = 4.2 # calibration, automate? (do in java, faster)
initth = 0
tfth = 0
globalpathposenum = 20 # just right
listener = None

def pathCallback(data): # local path
	global goalpose, lastpath
	
	lastpath = rospy.get_time()
	goalpose = False
	
def globalPathCallback(data):
	global targetx, targety, targetth , followpath
	n = len(data.poses)
	if n < 5:
		return
		
	if n-1 < globalpathposenum:
		p = data.poses[n-1] 
	else:
		p = data.poses[globalpathposenum]
	
	targetx = p.pose.position.x
	targety = p.pose.position.y
	quaternion = ( p.pose.orientation.x, p.pose.orientation.y,
	p.pose.orientation.z, p.pose.orientation.w )
	targetth = tf.transformations.euler_from_quaternion(quaternion)[2]
	
	followpath = True

def odomCallback(data):
	global odomx, odomy, odomth
	odomx = data.pose.pose.position.x
	odomy = data.pose.pose.position.y
	quaternion = ( data.pose.pose.orientation.x, data.pose.pose.orientation.y,
	data.pose.pose.orientation.z, data.pose.pose.orientation.w )
	odomth = tf.transformations.euler_from_quaternion(quaternion)[2]
	
	# determine direction (angle) on map
	global tfth, listener	 
	try:
		(trans,rot) = listener.lookupTransform('/map', '/odom', rospy.Time(0))
		quaternion = (rot[0], rot[1], rot[2], rot[3])
		tfth = tf.transformations.euler_from_quaternion(quaternion)[2]
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		pass	

def intialPoseCallback(data):
	# do full rotation on pose estimate, to hone-in amcl
	rospy.sleep(0.5) # let amcl settle
	oculusprimesocket.sendString("speed "+str(turnspeed) )
	oculusprimesocket.sendString("move right")
	rospy.sleep(secondspertwopi) # full rotation
	oculusprimesocket.sendString("move stop")
	oculusprimesocket.waitForReplySearch("<state> direction stop")
	# rospy.sleep(1) # let amcl settle << TODO: this is in separate thread so does nothing!
	
def goalCallback(d):
	global goalth, goalpose, lastpath

	# to prevent immediately rotating wrongly towards new goal direction 
	lastpath = rospy.get_time()
	goalpose = False

	# set goal angle
	data = d.goal.target_pose
	quaternion = ( data.pose.orientation.x, data.pose.orientation.y,
	data.pose.orientation.z, data.pose.orientation.w )
	goalth = tf.transformations.euler_from_quaternion(quaternion)[2]
	
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
	global odomx, odomy, odomth

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
		th = gth - tfth
		goalrotate = True
	else:
		th = tth
	
	# determine angle delta for move
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
				
def cleanup():
	oculusprimesocket.sendString("move stop")
	oculusprimesocket.sendString("state delete navigationenabled")


# MAIN

rospy.init_node('dwa_base_controller', anonymous=False)
listener = tf.TransformListener()
oculusprimesocket.connect()

rospy.Subscriber("odom", Odometry, odomCallback)
rospy.Subscriber("move_base/DWAPlannerROS/local_plan", Path, pathCallback)
rospy.Subscriber("move_base/goal", MoveBaseActionGoal, goalCallback)
rospy.Subscriber("move_base/status", GoalStatusArray, goalStatusCallback)
rospy.Subscriber("move_base/DWAPlannerROS/global_plan", Path, globalPathCallback)
rospy.Subscriber("initialpose", PoseWithCovarianceStamped, intialPoseCallback)
rospy.on_shutdown(cleanup)

while not rospy.is_shutdown():
	t = rospy.get_time()
	
	if t >= nextmove:
		# nextmove = t + listentime
		if goalseek and (followpath or goalpose):
			move(odomx, odomy, odomth, targetx, targety, targetth, goalth)
			nextmove = rospy.get_time() + listentime
			followpath = False
	
	if t - lastpath > 3:
		goalpose = True
	
	rospy.sleep(0.01)
		
cleanup()
