#!/usr/bin/env python

"""
relay ros twist messages to java movement commands
control speed via simple lookup table

if twist message contains both angular AND linear, break up move into timed segments

assumes robot is stopped before running this

"""

# rostopic pub -r 5 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.16, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'
# rostopic pub -r 5 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.15, y: 0, z: 0}, angular: {x: 0, y: 0, z: -0.5}}'
# rostopic pub -r 5 /cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: -0.8}}'
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
nextarcpath = 0
nextturn = 0
turnincrement = 0.65
pwm = 0

def twistCallback(data): # event handler for cmd_vel Twist messages
	"""
	check for same move as last time, return if so
	check for linear OR angular OR stop -- send move command
	check for linear AND angular -- send move linear, start timed move combos arc path
	"""
	global lastmove, checkmove, lastlinear, lastangular, direction, nextarcpath, turnincrement, nextturn
	lastmove = rospy.get_time()
	checkmove = True
	
	# skip if same move as last time
	if data.linear.x == lastlinear and data.angular.z == lastangular:
		return

	if not data.linear.x == 0 and data.angular.z == 0:  # linear only
		if data.linear.x > 0:
			direction = "forward" 
		else:
			direction = "backward"
		lastlinear = data.linear.x
		lastangular = 0
		nextarcpath = 0
		nextturn = 0
		sendMove(lastlinear*1.25, 0, direction)

	elif not data.angular.z == 0 and data.linear.x == 0: # angular 
		if data.angular.z > 0: 
			direction = "left"
		else:
			direction = "right"
		lastangular = data.angular.z
		lastlinear = 0
		nextarcpath = 0
		if nextturn == 0:
			nextturn = lastmove + turnincrement
			sendMove(0, lastangular, direction)
			
	elif data.angular.z == 0 and data.linear.x == 0: # stop
		checkmove = False
		lastangular = 0
		lastlinear = 0
		nextarcpath = 0
		nextturn = 0
		direction = "stop"
		sendMove(0, 0, direction)
		
	else: # arc path
		lastlinear = data.linear.x 
		lastangular = data.angular.z 
		nextturn = 0
		x = lastlinear
		z = lastangular
		d = ""
		if direction == "left" or direction == "right":
			if lastlinear > 0:
				d = "forward" 
			else:
				d = "backward"
			increment = linearincrement()
			z = 0
		else:
			if lastangular > 0: 
				d = "left"
			else:
				d = "right"
			z = lastangular
			if z < 0.5:
				z=0.5
			increment = angularincrement()
			x = 0
		# if nextarcpath > lastmove:
			# nextarcpath = nextarcpath + increment
		# else:
			# nextarcpath = lastmove + increment
		if d == direction:
			sendmove(0,0,"stop")
			nextarcpath = lastmove
		else:
			sendMove(x, z, d)
			nextarcpath = lastmove + increment
			direction = d
		
		
def arcpath():
	global lastlinear, lastangular, direction, nextarcpath
	
	if not direction=="stop":
		socketclient.waitForReplySearch("<state> direction stop")
		if nextarcpath == 0: # in case cancelled by event handler thread
			return
		
	if direction == "forward" or direction == "backward":
		if lastangular > 0: 
			direction = "left"
		else:
			direction = "right"
		increment = angularincrement()

	else:
		if lastlinear > 0:
			direction = "forward" 
		else:
			direction = "backward"

		increment = linearincrement()
		
	z = lastangular
	if z < 0.5:
		z=0.5
		
	sendMove(lastlinear, z, direction)
	nextarcpath = rospy.get_time()+increment

def angularincrement():
	global lastangular
	return 0.3 + abs(lastangular * 0.8)
	
def linearincrement():
	global lastlinear, lastangular
	# return 0.1 + abs(lastlinear * 2 / (lastangular * 2) )
	return 0.4 + abs(lastlinear * 8 / (lastangular * 2) )
	
def turn():
	global lastangular, direction, nextturn, turnincrement
	sendMove(0,0,"stop")
	socketclient.waitForReplySearch("<state> direction stop")
	if not nextturn == 0: # in case cancelled by event handler thread
		nextturn = rospy.get_time()+turnincrement
		sendMove(0, lastangular, direction)
		
	
def sendMove(x, z, d):
	global pwm
	motorPWM = 0
			
	if d == "forward" or d == "backward":
		motorPWM = lookupLinearSpeed(abs(x))
	elif d == "left" or d == "right":
		motorPWM = lookupAngularSpeed(abs(z))
		
	if not nextarcpath == 0 or not nextturn == 0:
		motorPWM = int(motorPWM * 1.25)
		if motorPWM > 255:
			motorPWM = 255
		
	if not motorPWM == 0 and not motorPWM == pwm:
		socketclient.sendString("speed "+str(motorPWM))
		pwm = motorPWM
		
	socketclient.sendString("move "+d)

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
		motorpwm = 70
	elif rps <= 1.5:
		motorpwm = 100
	else:
		motorpwm = 255
	return motorpwm
	

# MAIN

rospy.init_node('base_controller', anonymous=False)
rospy.Subscriber("cmd_vel", Twist, twistCallback)
#rospy.Subscriber("turtle1/cmd_vel", Twist, twistCallback) # TODO: testing only
socketclient.sendString("odometrystart")
socketclient.sendString("state stopbetweenmoves true")

cmd_vel = rospy.Publisher('/cmd_vel', Twist)
while not rospy.is_shutdown():
	t = rospy.get_time()
	if checkmove and t - lastmove > 0.5: # stop if no recent commands
		checkmove = False
		lastangular = 0
		lastlinear = 0
		nextarcpath = 0
		nextturn = 0
		direction = "stop"
		sendMove(0, 0, direction)
	elif not nextturn == 0 and t >= nextturn:
		turn()
	elif not nextarcpath == 0 and t >= nextarcpath:
		arcpath()
	# rospy.sleep(0.05) 

#shutdown:
socketclient.sendString("odometrystop")
socketclient.sendString("state stopbetweenmoves false")
socketclient.sendString("move stop")
