#!/usr/bin/env python

import rospy, tf
import os, struct, re
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import oculusprimesocket
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from sensor_msgs.msg import LaserScan
import actionlib
from actionlib_msgs.msg import *

"""
sending info:
rosmapinfo (origin, size, resolution) on callback
rosamcl (x,y,th) on callback
    -also sending global path, scan data at this time, if exists
roscurrentgoal  (x,y,th)    on callback
   
change to:
send amcl on callback, + latest odom << change to amcloffset only???
roscurrentgoal, rosmapinfo unchanged

send global path (without amcl offset), scan data, odom every 0.5 sec

"""


lockfilepath = "/run/shm/map.raw.lock"
lastodomupdate = 0
odomx = 0
odomy = 0
odomth = 0
globalpath = []
scannum = 0
scanpoints = []
sendinfodelay = 1.0
lastsendinfo = 0
move_base = None
goalseek = False
xoffst = 0
yoffst = 0
thoffst = 0
recoveryrotate = False
goal = None
turnspeed = 100
secondspertwopi = 4.2 # calibration, automate? (do in java, faster)

def mapcallBack(data):
	global lockfilepath
	lockfilepath = "/run/shm/map.raw.lock"
	framefilepath ="/run/shm/map.raw"
	
	if os.path.exists(lockfilepath):
		return
	
	open(lockfilepath, 'w') # creates lockfile
	 
	framefile = open(framefilepath, 'w')
	packeddata = struct.pack('%sb' %len(data.data), *data.data)
	framefile.write(packeddata)
	framefile.close()

	if os.path.exists(lockfilepath):
		os.remove(lockfilepath)

	quaternion = ( data.info.origin.orientation.x, data.info.origin.orientation.y,
	data.info.origin.orientation.z, data.info.origin.orientation.w )
	th = tf.transformations.euler_from_quaternion(quaternion)[2]
		
	# width height res originx originy originth updatetime	
	s = "state rosmapinfo "+str(data.info.width)+","+str(data.info.height)+","
	s += str(data.info.resolution)+","+str(data.info.origin.position.x)+","
	s += str(data.info.origin.position.y)+","+str(th)+","+str(rospy.get_time())
	
	oculusprimesocket.sendString(s)
	oculusprimesocket.sendString("state rosmapupdated true")

def odomCallback(data):
	 global odomx, odomy, odomth
	 odomx = data.pose.pose.position.x
	 odomy = data.pose.pose.position.y
	 quaternion = ( data.pose.pose.orientation.x, data.pose.pose.orientation.y,
	 data.pose.pose.orientation.z, data.pose.pose.orientation.w )
	 odomth = tf.transformations.euler_from_quaternion(quaternion)[2]
		
def amclPoseCallback(data):
	global odomx, odomy, odomth, goalseek, xoffst, yoffst, thoffst
	
	if not goalseek:
		amclx = data.pose.pose.position.x
		amcly = data.pose.pose.position.y
		quaternion = ( data.pose.pose.orientation.x, data.pose.pose.orientation.y,
		data.pose.pose.orientation.z, data.pose.pose.orientation.w )
		amclth = tf.transformations.euler_from_quaternion(quaternion)[2]

		xoffst = amclx - odomx
		yoffst = amcly - odomy
		thoffst = amclth - odomth
		
		# s = "state rosamcl "
		# s += str(round(xoffst, 3))+","+str(round(yoffst,3))+","+str(round(thoffst,3))+","
		# s +=  str(round(odomx,3))+","+str(round(odomy,3))+","+str(round(odomth,3))
# 
		# oculusprimesocket.sendString(s)
		# 
		# if len(globalpath) > 0:
			# sendGlobalPath()
		# if len(scanpoints) > 0:
			# sendScan()
		
def feedbackCallback(d):
	global odomx, odomy, odomth, xoffst, yoffst, thoffst
	data = d.feedback.base_position.pose
	amclx = data.position.x
	amcly = data.position.y
	quaternion = ( data.orientation.x, data.orientation.y,
	data.orientation.z, data.orientation.w )
	amclth = tf.transformations.euler_from_quaternion(quaternion)[2]

	xoffst = amclx - odomx
	yoffst = amcly - odomy
	thoffst = amclth - odomth
	
def goalCallback(d):
	data = d.goal.target_pose
	x = data.pose.position.x
	y = data.pose.position.y
	quaternion = ( data.pose.orientation.x, data.pose.orientation.y,
	data.pose.orientation.z, data.pose.orientation.w )
	th = tf.transformations.euler_from_quaternion(quaternion)[2]	
	oculusprimesocket.sendString("state roscurrentgoal "+str(x)+","+str(y)+","+str(th))
	oculusprimesocket.sendString("messageclients new navigation goal received");
	
def globalPathCallback(data):
	global globalpath
	globalpath = data.poses
	recoveryrotate = False
	
def sendGlobalPath():
	global globalpath
	s = "state rosglobalpath "
	step = 5 
	size = len(globalpath)
	i = 0

	while i < size - step:
		s = s + str(round(globalpath[i].pose.position.x,2))+","
		s = s + str(round(globalpath[i].pose.position.y,2))+","
		i += step

	s = s + str(round(globalpath[size-1].pose.position.x,2))
	s = s + str(round(globalpath[size-1].pose.position.y,2))

	oculusprimesocket.sendString(s)

def publishinitialpose(str):
	s = str.split("_")
	x = float(s[0])
	y = float(s[1])
	th = float(s[2])
	
	pose = PoseWithCovarianceStamped()
	pose.header.stamp = rospy.Time.now()
	pose.header.frame_id = "map"
	
	pose.pose.pose.position.x = x
	pose.pose.pose.position.y = y
	pose.pose.pose.position.z = 0.0
	
	quat = tf.transformations.quaternion_from_euler(0, 0, th)
	pose.pose.pose.orientation.x = quat[0]
	pose.pose.pose.orientation.y = quat[1]
	pose.pose.pose.orientation.z = quat[2]
	pose.pose.pose.orientation.w = quat[3]
	pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
	
	initpose_pub.publish(pose)
		
def publishgoal(str):
	global move_base, goal
	
	s = str.split("_")
	x = float(s[0])
	y = float(s[1])
	th = float(s[2])
	
	goal = MoveBaseGoal()
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.header.frame_id = "map"
	
	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y = y
	goal.target_pose.pose.position.z = 0.0
	
	quat = tf.transformations.quaternion_from_euler(0, 0, th)
	goal.target_pose.pose.orientation.x = quat[0]
	goal.target_pose.pose.orientation.y = quat[1]
	goal.target_pose.pose.orientation.z = quat[2]
	goal.target_pose.pose.orientation.w = quat[3]

	move_base.send_goal(goal)

def scanCallback(data):
	global scannum, scanpoints
	scannum += 1
	if scannum < 5:
		return
	scannum=0
	scanpoints = data.ranges

def sendScan():
	global scanpoints
	
	s = "state rosscan "
	
	step = 8 
	size = len(scanpoints)
	i = 0
	while i < size-step:
		s += str(round(scanpoints[i],3))+","
		i += step
	s += str(round(scanpoints[size-1],3))
	oculusprimesocket.sendString(s)

def cleanup():
	oculusprimesocket.sendString("state delete navigationenabled")		
	oculusprimesocket.sendString("state delete roscurrentgoal")
	oculusprimesocket.sendString("state delete rosamcl")
	oculusprimesocket.sendString("state delete rosglobalpath")
	oculusprimesocket.sendString("state delete rosscan")
	oculusprimesocket.sendString("messageclients navigation disabled")	

# main

oculusprimesocket.connect()	

rospy.init_node('remote_nav', anonymous=False)

if os.path.exists(lockfilepath):
	os.remove(lockfilepath)
	
oculusprimesocket.sendString("state delete roscurrentgoal")
oculusprimesocket.sendString("state delete rosamcl")
oculusprimesocket.sendString("state delete rosglobalpath")
oculusprimesocket.sendString("state delete rosmapinfo")
oculusprimesocket.sendString("state delete rosscan")
	
rospy.Subscriber("map", OccupancyGrid, mapcallBack)
rospy.Subscriber("odom", Odometry, odomCallback)
rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, amclPoseCallback)
rospy.Subscriber("move_base/goal", MoveBaseActionGoal, goalCallback)
rospy.Subscriber("move_base/DWAPlannerROS/global_plan", Path, globalPathCallback) 
rospy.Subscriber("scan", LaserScan, scanCallback)
rospy.Subscriber("move_base/feedback", MoveBaseActionFeedback, feedbackCallback)
initpose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
# goal_pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)

move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
move_base.wait_for_server()
oculusprimesocket.sendString("messageclients navigation ready")
oculusprimesocket.sendString("state navigationenabled true")
rospy.on_shutdown(cleanup)

lasttext = ""

while not rospy.is_shutdown():
	s = oculusprimesocket.replyBufferSearch("<state> (rosinitialpose|rossetgoal|rosgoalcancel) ")
	if re.search("rosinitialpose", s):
		oculusprimesocket.sendString("state delete rosinitialpose")
		publishinitialpose(s.split()[2])
		recoveryrotate = False

	elif re.search("rossetgoal", s):
		oculusprimesocket.sendString("state delete rossetgoal")
		globalpath = []
		publishgoal(s.split()[2])
		goalseek = True
		recoveryrotate = False
	
	elif re.search("rosgoalcancel true", s):
		move_base.cancel_goal()
		goalseek = False
		oculusprimesocket.sendString("messageclients cancel navigation goal")
		oculusprimesocket.sendString("state delete roscurrentgoal")
		oculusprimesocket.sendString("state delete rosgoalcancel")
		globalpath = []
		recoveryrotate = False
		
	t = rospy.get_time()
	if t - lastsendinfo > sendinfodelay:
		lastsendinfo = t

		if len(scanpoints) > 0:
			sendScan()
			
		# if goalseek and not xoffst == None:
		s = "state rosamcl "
		s += str(round(xoffst, 3))+","+str(round(yoffst,3))+","+str(round(thoffst,3))+","
		s +=  str(round(odomx,3))+","+str(round(odomy,3))+","+str(round(odomth,3))

		oculusprimesocket.sendString(s)
		
		if len(globalpath) > 0:
			sendGlobalPath()
			
	if goalseek: 
		state = move_base.get_state()
		if state == GoalStatus.SUCCEEDED: # error if not seeking goal
			oculusprimesocket.sendString("messageclients navigation goal reached")
			oculusprimesocket.sendString("state delete roscurrentgoal")
			goalseek = False
		elif state == GoalStatus.ABORTED: 
			if not recoveryrotate:
				recoveryrotate = True
				oculusprimesocket.sendString("messageclients recovery rotation")
				oculusprimesocket.sendString("speed "+str(turnspeed) )
				oculusprimesocket.sendString("move right")
				rospy.sleep(secondspertwopi)
				oculusprimesocket.sendString("move stop")
				oculusprimesocket.waitForReplySearch("<state> direction stop")
				rospy.sleep(1)
				move_base.send_goal(goal)
			else:
				oculusprimesocket.sendString("messageclients navigation goal ABORTED")
				oculusprimesocket.sendString("state delete roscurrentgoal")
				goalseek = False
		
		
	rospy.sleep(0.01) # this really helps with cpu load
		
