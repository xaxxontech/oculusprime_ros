#!/usr/bin/env python

import rospy
import os, shutil
from sensor_msgs.msg import Image


filenum = 0
MAXFILES = 100
PATH = "/dev/shm/rosimgframes"  # filename format: frame number (integer) only, with no extension
imgnum = 0


def cleanup():
	rospy.sleep(3)
	deletefiles()
	rospy.loginfo("image_to_shm quit")

		
def deletefiles():
	if os.path.isdir(PATH):
		try:
			shutil.rmtree(PATH)
		except:
			pass
	

def imgCallBack(data):
	global filenum, imgnum
	
	#  imgnum += 1
	#  if data.width > 640 and imgnum %2 == 0: # skip every other high res frame
		#  return 
	
	oldfile = PATH+"/"+str(filenum-MAXFILES)

	if os.path.exists(oldfile):
		os.remove(oldfile)
	
	f = open(PATH+"/"+str(filenum), "w")
	f.write(data.data)
	f.close()

	filenum += 1
	

deletefiles() 
os.mkdir(PATH)

rospy.init_node('image_to_shm', anonymous=False)
rospy.loginfo("image_to_shm init")
rospy.Subscriber(rospy.get_param('~camera_topic', 'usb_cam/image_raw'), Image, imgCallBack) 
rospy.on_shutdown(cleanup)
print "using topic: "+rospy.get_param('~camera_topic', 'usb_cam/image_raw')

rospy.spin()
