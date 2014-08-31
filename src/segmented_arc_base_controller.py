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
import thread
from geometry_msgs.msg import Twist
from math import radians


lastlinear = 0
lastangular = 0
lastmove = 0
checkmove = False
direction = "stop"
nextarcpath = 0
nextturn = 0
turnincrement = 0.50
lastpwm = 0
lastd = ""
movebuffer = []
lastsendmove = 0
minsendmove = 0.5
buffertime = 0


def twistCallback(data): # event handler for cmd_vel Twist messages
	"""
	check for same move as last time, return if so
	check for linear OR angular OR stop -- send move command
	check for linear AND angular -- send move linear, start timed move combos arc path
	"""
	global lastmove, checkmove, lastlinear, lastangular, direction, nextarcpath, turnincrement, nextturn, buffertime
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
		move(lastlinear*1.25, 0, direction)

	elif not data.angular.z == 0 and data.linear.x == 0: # angular 
		if data.angular.z > 0: 
			direction = "left"
		else:
			direction = "right"
		lastangular = data.angular.z
		lastlinear = 0
		nextarcpath = 0
		if nextturn == 0:
			nextturn = lastmove + turnincrement + buffertime
			move(0, lastangular, direction)
			
	elif data.angular.z == 0 and data.linear.x == 0: # stop
		checkmove = False
		lastangular = 0
		lastlinear = 0
		nextarcpath = 0
		nextturn = 0
		direction = "stop"
		move(0, 0, direction)
		
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
		
		if d == direction:
			sendmove(0,0,"stop")
			nextarcpath = lastmove + buffertime
		else:
			move(x, z, d)
			nextarcpath = lastmove + increment + buffertime
			direction = d
		
		
def arcpath():
	global lastlinear, lastangular, direction, nextarcpath, buffertime
	
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
		
	#if increment < 0.3:
		#increment = 0.3

	move(lastlinear, z, direction)
	nextarcpath = rospy.get_time()+increment + buffertime

def angularincrement():
	global lastangular
	return 0.2 + abs(lastangular * 0.6)
	
def linearincrement():
	global lastlinear, lastangular
	if lastangular == 0: # div by 0 thread safe
		return 0
	else:
		return 0.4 + abs(lastlinear * 8 / (lastangular * 2) )
	
def turn():
	global lastangular, direction, nextturn, turnincrement, buffertime
	move(0,0,"stop")
	socketclient.waitForReplySearch("<state> direction stop")
	if not nextturn == 0: # in case cancelled by event handler thread
		nextturn = rospy.get_time()+turnincrement+buffertime
		move(0, lastangular, direction)
		
def move(x,z,d):
	global movebuffer
	movebuffer.append([x,z,d])
	
def sendMove(x, z, d):
	global lastpwm, lastd 
	
	motorPWM = 0			
	if d == "forward" or d == "backward":
		motorPWM = lookupLinearSpeed(abs(x))
	elif d == "left" or d == "right":
		motorPWM = lookupAngularSpeed(abs(z))
		
	if not nextarcpath == 0 or not nextturn == 0:
		motorPWM = int(motorPWM * 1.25)
		if motorPWM > 255:
			motorPWM = 255
	
	oktomove = True	
	if lastd == d and motorPWM == lastpwm and not d=="stop":
		oktomove = False
	lastd = d
		
	if not motorPWM == 0 and not motorPWM == lastpwm:
		socketclient.sendString("speed "+str(motorPWM))
	lastpwm = motorPWM
	
	if oktomove:
		socketclient.sendString("move "+d)

def bufferMoves(): # thread
	global movebuffer, lastsendmove, minsendmove, buffertime 
	while not rospy.is_shutdown():
		t = rospy.get_time()
		if len(movebuffer) > 0:
			if t > lastsendmove + minsendmove or movebuffer[len(movebuffer)-1][2] == "stop":
				m = movebuffer.pop()
				sendMove(m[0], m[1], m[2])
				lastsendmove = t
		buffertime = minsendmove-(t-lastsendmove) + len(movebuffer)*minsendmove
		if buffertime < 0:
			buffertime = 0
		rospy.sleep(0.01)
	cleanup()
			
			

def lookupLinearSpeed(mps): 
	"""max speed pwm 255 = 0.39m/s
	min speed pwm 30 = 0.12m/s
	sysvolts 11.88
	"""
	motorpwm = 0
	if mps <= 0.15:
		motorpwm = 50
	elif mps <= 0.2:
		motorpwm = 70
	elif mps <= 0.25:
		motorpwm = 85
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
		motorpwm = 40
	elif rps <= 0.5:
		motorpwm = 50
	elif rps <= 0.8:
		motorpwm = 60
	elif rps <= 1.0:
		motorpwm = 70
	elif rps <= 1.5:
		motorpwm = 100
	else:
		motorpwm = 255
	return motorpwm
	
def cleanup():
	socketclient.sendString("odometrystop")
	socketclient.sendString("state stopbetweenmoves false")
	socketclient.sendString("move stop")

# MAIN

rospy.init_node('base_controller', anonymous=False)
rospy.Subscriber("cmd_vel", Twist, twistCallback)
rospy.on_shutdown(cleanup)
socketclient.sendString("odometrystart")
socketclient.sendString("state stopbetweenmoves true")

cmd_vel = rospy.Publisher('/cmd_vel', Twist)

thread.start_new_thread( bufferMoves, ( ) )

while not rospy.is_shutdown():
	t = rospy.get_time()
	if checkmove and t - lastmove > 0.5: # stop if no recent commands
		checkmove = False
		lastangular = 0
		lastlinear = 0
		nextarcpath = 0
		nextturn = 0
		direction = "stop"
		move(0, 0, direction)
	elif not nextturn == 0 and t >= nextturn:
		turn()
	elif not nextarcpath == 0 and t >= nextarcpath:
		arcpath()
	# rospy.sleep(0.05) 

cleanup()
