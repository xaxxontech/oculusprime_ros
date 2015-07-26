#!/usr/bin/env python

import rospy, tf
import os, struct, re
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import oculusprimesocket

"""


"""

lockfilepath = "/run/shm/map.raw.lock"
scannum = 0
scanpoints = []
sendinfodelay = 1.0
lastsendinfo = 0

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

def scanCallback(data):
	global firstscan
	firstscan.unregister()
	oculusprimesocket.sendString("state navsystemstatus mapping") 
	
	# global scannum, scanpoints
	# scannum += 1
	# if scannum < 5:
		# return
	# scannum=0
	# scanpoints = data.ranges

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

oculusprimesocket.sendString("state delete roscurrentgoal")
oculusprimesocket.sendString("state delete rosamcl")
oculusprimesocket.sendString("state delete rosglobalpath")
oculusprimesocket.sendString("state delete rosmapinfo")
oculusprimesocket.sendString("state delete rosscan")
oculusprimesocket.sendString("state delete rosmapupdated")

# oculusprimesocket.sendString("state odomturndpms 0.06")  # degrees per ms
# oculusprimesocket.sendString("state odomturnpwm 65")  # measured, approx starting point smooth floor
# oculusprimesocket.sendString("speed 150") # linear speed

rospy.init_node('map_remote', anonymous=False)

if os.path.exists(lockfilepath):
	os.remove(lockfilepath)
	
rospy.Subscriber("map", OccupancyGrid, mapcallBack)
firstscan = rospy.Subscriber("scan", LaserScan, scanCallback)



while not rospy.is_shutdown():
		
	t = rospy.get_time()
	if t - lastsendinfo > sendinfodelay:
		lastsendinfo = t

		if len(scanpoints) > 0:
			sendScan()
		
	rospy.sleep(0.01) # this really helps with cpu load
		
