#!/usr/bin/env python

import rospy
import oculusprimesocket
from std_msgs.msg import String


def callback(msg):
	if (msg.data == "disconnected"):
		oculusprimesocket.sendString("driverexit")  

	

"""main"""
oculusprimesocket.connect()	
rospy.init_node('webrtc_status_listener', anonymous=True)
oculusprimesocket.sendString("log webrtc_status_listener.py connected")  
rospy.Subscriber("webrtcstatus", String, callback)

rospy.spin()
