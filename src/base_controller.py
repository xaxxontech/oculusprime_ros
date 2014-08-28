#!/usr/bin/env python

"""
relay ros twist messages to java movement commands
control speed via simple lookup table
"""

# rostopic pub -r 5 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.15, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'
# rosrun turtlesim turtle_teleop_key

import rospy
import socketclient
from geometry_msgs.msg import Twist
from math import radians


lastlinear = 0
lastangular = 0
lastmove = 0
checkmove = False
direction = "stop"


def twistCallback(data): # event handler for cmd_vel Twist messages
	global lastmove, checkmove, lastlinear, lastangular, direction
	lastmove = rospy.get_time()
	checkmove = True
	if data.linear.x > 0:  # forward (overrides angular)
		if not data.linear.x == lastlinear:
			direction = "forward" 
			lastlinear = data.linear.x
			lastangular = 0
			sendMove()
		
	elif data.linear.x < 0: # backward (overrides angular)
		if not data.linear.x == lastlinear:
			direction = "backward"
			lastlinear = data.linear.x
			lastangular = 0
			sendMove()
			
	elif data.angular.z > 0: # left
		if not data.angular.z == lastangular: 
			direction = "left"
			lastangular = data.angular.z
			lastlinear = 0
			sendMove()
			
	elif data.angular.z < 0: # right
		if not data.angular.z == lastangular: 
			direction = "right"
			lastangular = data.angular.z
			lastlinear = 0
			sendMove()
			
	else: # stop
		checkmove = False
		lastangular = 0
		lastlinear = 0
		direction = "stop"
		sendMove()


def sendMove():
	global lastlinear, lastangular, direction
	motorPWM = 0

	if not lastlinear == 0:
		motorPWM = lookupLinearSpeed(abs(lastlinear))
	elif not lastangular == 0:
		motorPWM = lookupAngularSpeed(abs(lastangular))
	
	if not motorPWM == 0:
		socketclient.sendString("speed "+str(motorPWM))
	socketclient.sendString("move "+direction)

def lookupLinearSpeed(mps): 
	"""max speed pwm 255 = 0.39m/s
	min speed pwm 30 = 0.12m/s
	sysvolts 11.88
	"""
	motorpwm = 0
	if mps <= 0.15:
		motorpwm = 30
	elif mps <= 0.2:
		motorpwm = 40
	elif mps <= 0.25:
		motorpwm = 50
	elif mps <= 0.3:
		motorpwm = 100
	else:
		motorpwm = 255
	return motorpwm

def lookupAngularSpeed(rps): 
	"""max speed pwm 255 = 2.15r/s
	min speed pwm 30 = 0.36r/s
	sysvolts 11.88
	"""
	motorpwm = 0
	if rps <= 0.2:
		motorpwm = 30
	elif rps <= 0.5:
		motorpwm = 40
	elif rps <= 0.8:
		motorpwm = 50
	elif rps <= 1.0:
		motorpwm = 50
	elif rps <= 1.5:
		motorpwm = 100
	else:
		motorpwm = 255
	return motorpwm

# MAIN

rospy.init_node('base_controller', anonymous=False)
rospy.Subscriber("cmd_vel", Twist, twistCallback)
rospy.Subscriber("turtle1/cmd_vel", Twist, twistCallback) # TODO: testing only

# publish stop if no updates
cmd_vel = rospy.Publisher('/cmd_vel', Twist)
while not rospy.is_shutdown():
	t = rospy.get_time()
	if checkmove and t - lastmove > 0.5: 
		cmd_vel.publish(Twist())  
	rospy.sleep(0.1) 

# stop wheels if necessary on shutdown
if not lastlinear==0 or not lastangular==0:
	socketclient.sendString("move stop")

