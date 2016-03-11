#!/usr/bin/env python

import rospy, oculusprimesocket, math, re
from geometry_msgs.msg import Twist

lastlinear = 0
lastangular = 0

def twistCallback(data):
	global lastlinear, lastangular

	linear = data.linear.x
	angular = data.angular.z
	
	# determine if moving
	oculusprimesocket.clearIncoming()
	oculusprimesocket.sendString("state moving")
	s = oculusprimesocket.waitForReplySearch("<state> moving")
	moving = True
	if (re.search("false",s)):
		moving = False

	# continue only if not moving or different movement from last
	if linear == lastlinear and angular == lastangular:
		if (linear == 0 and angular == 0) or moving:
			return
	
	lastlinear = linear
	lastangular = angular	
	
	cmd = None

	if linear == 0 and angular == 0:
		cmd = "move stop"
		
	elif linear > 0 and angular == 0:
		cmd = "forward 0.3"
		
	elif linear < 0 and angular == 0:
		cmd = "backward 0.3"
		
	elif linear == 0 and angular > 0:
		cmd = "left 40"
		
	elif linear == 0 and angular < 0:
		cmd = "right 40"
		
	elif linear > 0 and not angular == 0:  # forward arc
		distance = "0.33" 
		angle = str(int(math.degrees(angular))/3)
		cmd = "arcmove "+distance+" "+angle
	
	elif linear < 0 and not angular == 0:  # backwards arc
		distance = "-0.33" 
		angle = str(int(math.degrees(angular))/3)
		cmd = "arcmove "+distance+" "+angle
				
	if not cmd == None:
		oculusprimesocket.sendString(cmd)
		# print (str(linear)+", "+str(angular)+", "+cmd)


# Main	

# initialize node
rospy.init_node('cmd_vel_listener', anonymous=False)

# connect to oculusprime java server
oculusprimesocket.connect()
oculusprimesocket.sendString("log cmd_vel_listener.py connected") 
oculusprimesocket.sendString("odometrystart") 

# twist message event listener
rospy.Subscriber("cmd_vel", Twist, twistCallback)

rospy.spin()

