#!/usr/bin/env python

import rospy, tf
import os, struct
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import oculusprimesocket

'''
rostopic info /map
Type: nav_msgs/OccupancyGrid
Publishers: 
 * /map_server (http://megatron:54907/)
Subscribers: 
 * /move_base (http://megatron:51295/)
 * /rviz_1421300705033674927 (http://imac:52841/)

rosmsg show nav_msgs/OccupancyGrid
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
nav_msgs/MapMetaData info
  time map_load_time
  float32 resolution
  uint32 width
  uint32 height
  geometry_msgs/Pose origin
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
int8[] data

data (pixel) contents: -1 or 0 or 100

'''

lockfilepath = "/run/shm/map.raw.lock"
lastodomupdate = 0

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
	global lastodomupdate
	odomx = data.pose.pose.position.x
	odomy = data.pose.pose.position.y
	quaternion = ( data.pose.pose.orientation.x, data.pose.pose.orientation.y,
	data.pose.pose.orientation.z, data.pose.pose.orientation.w )
	odomth = tf.transformations.euler_from_quaternion(quaternion)[2]
	
	if rospy.get_time() - lastodomupdate > 0.5:
		# odomx odomy odomth
		oculusprimesocket.sendString("state rosodom "+str(odomx)+"_"+str(odomy)+"_"+str(odomth))
		lastodomupdate = rospy.get_time()

# main
	
rospy.init_node('remote_nav', anonymous=False)

if os.path.exists(lockfilepath):
	os.remove(lockfilepath)
	
rospy.Subscriber("map", OccupancyGrid, mapcallBack)
rospy.Subscriber("odom", Odometry, odomCallback)
oculusprimesocket.connect()

rospy.spin()
