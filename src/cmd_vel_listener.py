#!/usr/bin/env python

import rospy, oculusprimesocket, math, re, thread, time
from geometry_msgs.msg import Twist

lastlinear = 0
lastangular = 0
lastmove = 0
smoothingdelay = 0.3
twistid = 0


def twistCallback(data):
	global twistid

	linear = data.linear.x
	angular = data.angular.z
	
	twistid = rospy.Time.now()
	thread.start_new_thread(waitifnecessary, (linear, angular, twistid) )
	# print twistid


def waitifnecessary(linear, angular, threadid):

	if linear == 0 and angular == 0: # always stop immediately
		move(linear, angular)
		return
		
	while twistid == threadid and smoothingdelay - (rospy.Time.now()-lastmove).to_sec() > 0:
		rospy.sleep(0.01)

	if twistid == threadid:
		move(linear, angular)
	
	
def move(linear, angular):
	global lastlinear, lastangular, lastmove
	
	lastmove = rospy.Time.now()

	# don't send repeat stop commands
	if linear == lastlinear and angular == lastangular and linear == 0 and angular == 0:
		return

	lastlinear = linear
	lastangular = angular	

	cmd = None
	
	d = "0.3" # .3
	a = "25" # 40
	arcmult = 3 # 3

	if linear == 0 and angular == 0:
		cmd = "move stop"
		
	elif linear > 0 and angular == 0:
		cmd = "forward "+d
		
	elif linear < 0 and angular == 0:
		cmd = "backward "+d
		
	elif linear == 0 and angular > 0:
		cmd = "left "+a
		
	elif linear == 0 and angular < 0:
		cmd = "right "+a
		
	elif linear > 0 and not angular == 0:  # forward arc
		angle = str(int(math.degrees(angular))/arcmult)
		cmd = "arcmove "+d+" "+angle
	
	elif linear < 0 and not angular == 0:  # backwards arc
		angle = str(int(math.degrees(angular))/arcmult)
		cmd = "arcmove -"+d+" "+angle
				
	if not cmd == None:
		oculusprimesocket.sendString(cmd)
		# print (str(linear)+", "+str(angular)+", "+cmd)
		
		
def cleanup():
	oculusprimesocket.sendString("odometrystop")
	oculusprimesocket.sendString("log cmd_vel_listener.py disconnecting")  # goodbye 


# Main	

# initialize node
rospy.init_node('cmd_vel_listener', anonymous=False)
rospy.on_shutdown(cleanup)
lastmove = rospy.Time.now()

# connect to oculusprime java server
oculusprimesocket.connect()
oculusprimesocket.sendString("log cmd_vel_listener.py connected") 
oculusprimesocket.sendString("odometrystart") 

# twist message event listener
rospy.Subscriber("cmd_vel", Twist, twistCallback)

rospy.spin()
