#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import pi, radians, sqrt, pow


x = 0
y = 0
th = 0


def shutdown():
	# Always stop the robot when shutting down the node.
	cmd_vel.publish(Twist())
	rospy.sleep(1)
	
def odomCallback(data):
	global x, y, th
	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	th = data.pose.pose.orientation.z * pi
	if th == -pi:
		th = pi
	elif abs(th) == 2*pi:
		th = 0


# MAIN

rospy.init_node('out_and_back', anonymous=False)
rospy.on_shutdown(shutdown)
cmd_vel = rospy.Publisher('/cmd_vel', Twist)

rate = 100
r = rospy.Rate(rate)

linear_speed = 0.2
goal_distance = 0.5
angular_speed = 1.0
angular_padding = radians(2.5)
linear_padding = 0.03
goal_angle = pi  # 180 deg

rospy.Subscriber("odom", Odometry, odomCallback) 
rospy.sleep(1) # wait for odometry info to update

# Loop once for each leg of the trip
for i in range(2):
	# Initialize the movement command
	move_cmd = Twist()
	
	# Set the movement command to forward motion
	move_cmd.linear.x = linear_speed
	
	x_start = x
	y_start = y
	th_start = th
	distance = 0
	
	# Enter the loop to move along a side
	while distance+linear_padding < goal_distance and not rospy.is_shutdown():
		cmd_vel.publish(move_cmd)
		r.sleep()
		distance = sqrt(pow((x - x_start), 2) + pow((y - y_start), 2))

	# Stop the robot before the rotation
	move_cmd = Twist()
	cmd_vel.publish(move_cmd)
	rospy.sleep(1)
	
	# Set the movement command to a rotation
	move_cmd.angular.z = angular_speed
	
	last_angle = th_start
	turn_angle = 0
	
	while abs(turn_angle + angular_padding) < goal_angle and not rospy.is_shutdown():
		# Publish the Twist message and sleep 1 cycle         
		cmd_vel.publish(move_cmd)
		r.sleep()
		
		# Compute the amount of rotation since the last loop
		delta_angle = th - last_angle
		if delta_angle == -pi:
			delta_angle = pi
		elif abs(delta_angle) == 2*pi:
			delta_angle = 0
		
		# Add to the running total
		turn_angle += delta_angle
		last_angle = th
		
	# Stop the robot before the next leg
	move_cmd = Twist()
	cmd_vel.publish(move_cmd)
	rospy.sleep(1)
	
# Stop the robot for good
cmd_vel.publish(Twist())
		
# Always stop the robot when shutting down the node.
cmd_vel.publish(Twist())
rospy.sleep(1)
 


