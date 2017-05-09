#!/usr/bin/env python

import rospy
import oculusprimesocket


def cleanup():
	oculusprimesocket.sendString("malgcommand a 0") # turn off motor
	oculusprimesocket.sendString("state neatopwm 0")
	oculusprimesocket.sendString("messageclients lidar off")


rospy.init_node('neato_rpm_controller', anonymous=False)
rospy.on_shutdown(cleanup)
oculusprimesocket.connect()

oculusprimesocket.sendString("malgcommand a 233") # temporary
oculusprimesocket.sendString("state neatopwm 233") 
oculusprimesocket.sendString("messageclients lidar on")


while not rospy.is_shutdown():
		
	rospy.sleep(0.01) # was 0.01
