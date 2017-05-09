#!/usr/bin/env python

import time
import rospy 
from sensor_msgs.msg import CameraInfo

num = 0
lastavg = 0

def camInfoCallBack(data):
	tstamp = (int(str(data.header.stamp)))/1000000.0
	tnow = time.time() * 1000 # ms
	print(str(data.header.seq) +" " +'%f' %(tstamp)+" "+'%f' %(tnow)+" "+str(tstamp-tnow)) 

	# global num, lastavg
	# num += 1
	# 
	# print(num)
	# 
	# if lastavg == 0:
		# lastavg = time.time() * 1000
# 
	# if num>100:
		# tnow = time.time() * 1000
		# avg = 100/((tnow-lastavg)/1000)
		# print ("FPS: "+str(avg))
		# lastavg = tnow
		# num = 0
	

# MAIN

rospy.init_node('read_raw_timestamps', anonymous=False)
rospy.Subscriber("camera/depth/camera_info", CameraInfo, camInfoCallBack)

rospy.spin()
