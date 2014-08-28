#!/usr/bin/env python
import rospy
import oculusprime
import re
from geometry_msgs.msg import Twist

#callbackrunning = False

# requires 	
# broadcasting on /turtle1/cmd_vel
# msg type: geometry_msgs/Twist:
	#geometry_msgs/Vector3 linear
	  #float64 x   << is +2.0 for forward, -2.0 for backward
	  #float64 y
	  #float64 z
	#geometry_msgs/Vector3 angular
	  #float64 x
	  #float64 y
	  #float64 z  << is -2.0 for left, +2.0 for right
  
def callback(data):
	oculusprime.sendString("state moving")
	s = oculusprime.waitForReplySearch("<state> moving")
	#print(s);
	if (re.search("false",s)):
		#global callbackrunning
		#if not callbackrunning:
			#callbackrunning = True
		if data.linear.x > 0: # forward
			oculusprime.sendString("movedistance forward 0.1")
		elif data.linear.x < 0: # backward
			oculusprime.sendString("movedistance backward 0.1")
		elif data.angular.z > 0: # left
			oculusprime.sendString("rotate left 20")
		elif data.angular.z < 0: # right
			oculusprime.sendString("rotate right 20")
			#callbackrunning = False

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("turtle1/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()


