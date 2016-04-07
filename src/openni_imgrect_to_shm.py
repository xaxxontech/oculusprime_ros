#!/usr/bin/env python

import rospy
import os
from sensor_msgs.msg import Image

'''
rostopic info /camera/depth/image_rect
Type: sensor_msgs/Image

rosmsg show sensor_msgs/Image
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
'''

def imgrect_callBack(data):
	lockfilepath = "/run/shm/xtion.raw.lock"
	framefilepath ="/run/shm/xtion.raw"
	
	if os.path.exists(lockfilepath):
		return
	
	open(lockfilepath, 'w') # creates lockfile
	 
	framefile = open(framefilepath, 'w')
	framefile.write(data.data)
	framefile.close()

	if os.path.exists(lockfilepath):
		os.remove(lockfilepath)
	 
	# print data.width  # yeilds 640 default
	# print data.is_bigendian # yeilds 0 default
	# print data.encoding # yeilds 32FC1
	# rospy.signal_shutdown("shutdown")
	

rospy.init_node('openni_imgrect_to_shm', anonymous=False)
rospy.Subscriber("camera/depth/image_raw", Image, imgrect_callBack)

rospy.spin()
