#!/usr/bin/env python

import rospy, tf
import os, struct, re
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import oculusprimesocket
from move_base_msgs.msg import MoveBaseActionGoal
from sensor_msgs.msg import LaserScan

"""


"""


lockfilepath = "/run/shm/map.raw.lock"
lastodomupdate = 0
odomx = 0
odomy = 0
odomth = 0
amclx = 0
amcly = 0
amclth = 0
globalpath = []
scannum = 0
scanpoints = []

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
	s = "state rosmapinfo "+str(data.info.width)+"_"+str(data.info.height)+"_"
	s += str(data.info.resolution)+"_"+str(data.info.origin.position.x)+"_"
	s += str(data.info.origin.position.y)+"_"+str(th)+"_"+str(rospy.get_time())
	
	oculusprimesocket.sendString(s)
		 
	# print data.width  # yeilds 640 default
	# print data.is_bigendian # yeilds 0 default
	# print data.encoding # yeilds 32FC1
	# rospy.signal_shutdown("yo man")

def odomCallback(data):
	 global odomx, odomy, odomth
	 odomx = data.pose.pose.position.x
	 odomy = data.pose.pose.position.y
	 quaternion = ( data.pose.pose.orientation.x, data.pose.pose.orientation.y,
	 data.pose.pose.orientation.z, data.pose.pose.orientation.w )
	 odomth = tf.transformations.euler_from_quaternion(quaternion)[2]
		
def amclPoseCallback(data):
	global amclx, amcly, amclth, globalpath
	amclx = data.pose.pose.position.x
	amcly = data.pose.pose.position.y
	quaternion = ( data.pose.pose.orientation.x, data.pose.pose.orientation.y,
	data.pose.pose.orientation.z, data.pose.pose.orientation.w )
	amclth = tf.transformations.euler_from_quaternion(quaternion)[2]

	oculusprimesocket.sendString("state rosamcl "+str(amclx)+"_"+str(amcly)+"_"+str(amclth))
	if len(globalpath) > 0:
		sendGlobalPath()
	if len(scanpoints) > 0:
		sendScan()
	
def goalCallback(d):
	data = d.goal.target_pose
	x = data.pose.position.x
	y = data.pose.position.y
	quaternion = ( data.pose.orientation.x, data.pose.orientation.y,
	data.pose.orientation.z, data.pose.orientation.w )
	th = tf.transformations.euler_from_quaternion(quaternion)[2]	
	oculusprimesocket.sendString("state roscurrentgoal "+str(x)+"_"+str(y)+"_"+str(th))
	
def globalPathCallback(data):
	global globalpath
	globalpath = data.poses
	
def sendGlobalPath():
	global amclx, amcly, amclth, odomx, odomy, odomth, globalpath
	s = "state rosglobalpath "
	step = 5 
	size = len(globalpath)
	i = 0
	xoffst = str(round(amclx - odomx, 3));
	yoffst = str(round(amcly - odomy, 3));
	thoffst = str(round(amclth - odomth, 3));
	s += xoffst+","+yoffst+","+thoffst+"_"
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
	s = str.split("_")
	x = float(s[0])
	y = float(s[1])
	th = float(s[2])
	
	goal = MoveBaseActionGoal()
	goal.header.stamp = rospy.Time.now()
	goal.goal.target_pose.header.stamp = rospy.Time.now()
	goal.goal.target_pose.header.frame_id = "map"
	
	goal.goal.target_pose.pose.position.x = x
	goal.goal.target_pose.pose.position.y = y
	goal.goal.target_pose.pose.position.z = 0.0
	
	quat = tf.transformations.quaternion_from_euler(0, 0, th)
	goal.goal.target_pose.pose.orientation.x = quat[0]
	goal.goal.target_pose.pose.orientation.y = quat[1]
	goal.goal.target_pose.pose.orientation.z = quat[2]
	goal.goal.target_pose.pose.orientation.w = quat[3]

	goal_pub.publish(goal)

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
initpose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
goal_pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)


while not rospy.is_shutdown():
	s = oculusprimesocket.replyBufferSearch("<state> (rosinitialpose|rossetgoal) ")
	if re.search("rosinitialpose", s):
		oculusprimesocket.sendString("state delete rosinitialpose")
		publishinitialpose(s.split()[2])

	elif re.search("rossetgoal", s):
		oculusprimesocket.sendString("state delete rossetgoal")
		publishgoal(s.split()[2])
		
	rospy.sleep(0.01) # this really helps with cpu load
		
