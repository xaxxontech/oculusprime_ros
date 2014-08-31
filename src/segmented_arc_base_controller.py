#!/usr/bin/env python

"""
listen to cmd_vel messages for n seconds, determine endpoint
use rotate and movedistance commands to get there, repeat
assumes robot is stopped before running this

"""

# rostopic pub -r 5 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.16, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'
# rostopic pub -r 5 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.15, y: 0, z: 0}, angular: {x: 0, y: 0, z: -0.5}}'
# rostopic pub -r 5 /cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: -0.8}}'
# rosrun turtlesim turtle_teleop_key

import rospy
import socketclient
from geometry_msgs.msg import Twist
from math import degrees

distx = 0
angz = 0
lastcmdvel = 0
lastmove = 0
listentime = 2.5 # constant, seconds

def twistCallback(data): # event handler for cmd_vel Twist messages
	global distx, angz, lastmove, lastcmdvel
	t = rospy.get_time()
	if lastcmdvel == 0:
		lastcmdvel = t
		return
	
	distx += data.linear.x * (t-lastcmdvel)
	angz += data.angular.z * (t-lastcmdvel)
	lastcmdvel = t
	
def move(x, z):
	if z > 0:
		socketclient.sendString("left "+str(abs(int(degrees(z)))) )
		socketclient.waitForReplySearch("<state> direction stop")
	elif z < 0:
		socketclient.sendString("right "+str(abs(int(degrees(z)))) )
		socketclient.waitForReplySearch("<state> direction stop")

	if x > 0:
		socketclient.sendString("forward "+str(abs(x)))
		socketclient.waitForReplySearch("<state> direction stop")
	elif x < 0:
		socketclient.sendString("backward "+str(abs(x)))
		socketclient.waitForReplySearch("<state> direction stop")
	
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

while not rospy.is_shutdown():
	t = rospy.get_time()
	if t - lastmove >= listentime:
		x = distx
		z = angz
		distx = 0
		angz = 0
		lastmove = t
		move(x, z)


cleanup()
