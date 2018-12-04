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
from std_srvs.srv import Empty

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
# docked = False

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
	oculusprimesocket.sendString("state delete rosgoalstatus")
	oculusprimesocket.sendString("messageclients new navigation goal received");
	
def globalPathCallback(data):
	global globalpath
	globalpath = data.poses
	recoveryrotate = False
	
def sendGlobalPath(path):
	s = "state rosglobalpath "
	step = 5 
	size = len(path)
	i = 0

	while i < size - step:
		s = s + str(round(path[i].pose.position.x,2))+","
		s = s + str(round(path[i].pose.position.y,2))+","
		i += step

	s = s + str(round(path[size-1].pose.position.x,2))
	s = s + str(round(path[size-1].pose.position.y,2))

	oculusprimesocket.sendString(s)

def publishinitialpose(str):
	# global docked
	
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

	# if not docked: 
		# pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
	# docked = False # only skip covariance set on initial docked position

	pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]


	initpose_pub.publish(pose)
		
def publishgoal(str):
	global move_base, goal
	
	s = str.split(",")
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
	size = len(scanpoints)
	
	step = 8  # vga depth cam, size typically 640
	if (size < 600):  # xaxxon lidar
		# step = 3
		step = size/100
	i = 0
	while i < size-step:
		s += str(round(scanpoints[i],3))+","
		i += step
		
	try:
		s += str(round(scanpoints[size-1],3))
	except IndexError: # rare lidar scan size error
		oculusprimesocket.sendString("messageclients remote_nav.py IndexError")
		
	oculusprimesocket.sendString(s)

def cleanup():
	oculusprimesocket.sendString("state delete roscurrentgoal")
	oculusprimesocket.sendString("state delete rosamcl")
	oculusprimesocket.sendString("state delete rosglobalpath")
	oculusprimesocket.sendString("state delete rosscan")
	oculusprimesocket.sendString("log remote_nav.py disconnecting") 	

def goalcancel():
	global goalseek, recoveryrotate
	
	move_base.cancel_goal()
	goalseek = False
	oculusprimesocket.sendString("messageclients cancel navigation goal")
	oculusprimesocket.sendString("state delete roscurrentgoal")
	oculusprimesocket.sendString("state delete rosgoalcancel")
	globalpath = []
	recoveryrotate = False
	
# main

oculusprimesocket.connect()	

rospy.init_node('remote_nav', anonymous=False)

if os.path.exists(lockfilepath):
	os.remove(lockfilepath)
	
oculusprimesocket.sendString("log remote_nav.py connected")  
oculusprimesocket.sendString("state delete roscurrentgoal")
oculusprimesocket.sendString("state delete rosamcl")
oculusprimesocket.sendString("state delete rosglobalpath")
oculusprimesocket.sendString("state delete rosmapinfo")
oculusprimesocket.sendString("state delete rosscan")
oculusprimesocket.sendString("state delete rosgoalstatus")

# oculusprimesocket.sendString("state dockstatus")
# if oculusprimesocket.waitForReplySearch("<state> dockstatus").split()[3] == "docked":
	# docked = True
	
rospy.Subscriber("map", OccupancyGrid, mapcallBack)
rospy.Subscriber("odom", Odometry, odomCallback)
rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, amclPoseCallback)
rospy.Subscriber("move_base/goal", MoveBaseActionGoal, goalCallback)
rospy.Subscriber("move_base/DWAPlannerROS/global_plan", Path, globalPathCallback) 
rospy.Subscriber("scan", LaserScan, scanCallback)
rospy.Subscriber("move_base/feedback", MoveBaseActionFeedback, feedbackCallback)
initpose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
rospy.on_shutdown(cleanup)

move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
move_base.wait_for_server()
rospy.sleep(0.5)
if not rospy.is_shutdown():
	oculusprimesocket.sendString("messageclients navigation ready") 
	oculusprimesocket.sendString("state navsystemstatus running") 

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
		goalcancel()
		
	t = rospy.get_time()
	if t - lastsendinfo > sendinfodelay:
		lastsendinfo = t

		if len(scanpoints) > 0:
			sendScan()
			
		# if goalseek and not xoffst == None:
		s = "state rosamcl "
		s += str(round(xoffst, 3))+","+str(round(yoffst,3))+","+str(round(thoffst,3))+","
		s += str(round(odomx,3))+","+str(round(odomy,3))+","+str(round(odomth,3))

		oculusprimesocket.sendString(s)
		
		if len(globalpath) > 0:
			sendGlobalPath(globalpath)
			
	if goalseek: 
		state = move_base.get_state()
		if state == GoalStatus.SUCCEEDED: # error if not seeking goal
			oculusprimesocket.sendString("messageclients navigation goal reached")
			oculusprimesocket.sendString("state rosgoalstatus succeeded") # having this below the one below may have caused null pointer
			oculusprimesocket.sendString("state delete roscurrentgoal")
			goalseek = False
		elif state == GoalStatus.ABORTED: 
			if not recoveryrotate:
				recoveryrotate = True
				
				#### recovery routine

				oculusprimesocket.sendString("messageclients navigation recovery")
				oculusprimesocket.clearIncoming()
				
				# cancel goal, clear costmaps, generally reset as much as possible
				move_base.cancel_goal()
				rospy.wait_for_service('/move_base/clear_costmaps')
				rospy.ServiceProxy('/move_base/clear_costmaps', Empty)()

				# wait for cpu
				rospy.sleep(2) 
				oculusprimesocket.sendString("waitforcpu")
				oculusprimesocket.waitForReplySearch("<state> waitingforcpu false")
				
				# check if cancelled by user while waiting
				oculusprimesocket.sendString("state rosgoalcancel") 
				s = oculusprimesocket.waitForReplySearch("<state> rosgoalcancel") 
				if re.search("rosgoalcancel true", s):
					oculusprimesocket.sendString("user cancelled nav goal")
					goalcancel()
					continue
			
				# resend pose, full rotate
				# publishinitialpose(str(xoffst+odomx)+"_"+str(yoffst+odomy)+"_"+str(thoffst+odomth))
				# oculusprimesocket.sendString("right 360")
				# oculusprimesocket.waitForReplySearch("<state> direction stop")
				
				# back up a bit
				oculusprimesocket.sendString("backward 0.25")
				oculusprimesocket.waitForReplySearch("<state> direction stop")
				
				# wait for cpu
				rospy.sleep(2) 
				oculusprimesocket.sendString("waitforcpu")
				oculusprimesocket.waitForReplySearch("<state> waitingforcpu false")

				# check if cancelled by user while waiting
				oculusprimesocket.sendString("state rosgoalcancel") 
				s = oculusprimesocket.waitForReplySearch("<state> rosgoalcancel") 
				if re.search("rosgoalcancel true", s):
					oculusprimesocket.sendString("user cancelled nav goal")
					goalcancel()
					continue

				move_base.send_goal(goal) # try once more

				#### end of recovery routine

				
			else:
				oculusprimesocket.sendString("messageclients navigation goal ABORTED")
				oculusprimesocket.sendString("state rosgoalstatus aborted")
				oculusprimesocket.sendString("state delete roscurrentgoal")
				goalseek = False
		
		
	rospy.sleep(0.01) # non busy wait
		
