#!/usr/bin/env python

"""
connect to Oculus Prime Server Application
poll server for odometry data
broadcast tranform between base_link and odom frames
"""

from math import radians, sin, cos
import rospy, tf
from nav_msgs.msg import Odometry
import oculusprimesocket


# lastupdate = 0
# updateinterval = 0.25
pos = [0.0, 0.0, 0.0]
before = 0
now = 0
default_lag = 0.035 # 0.075 = xtion (using device time)  0.035 = astra (using system time)


def broadcast(s):
	global before, pos, now
	now = rospy.Time.now() - rospy.Duration(rospy.get_param('~odom_lag', default_lag))
				# 0.05 subtract socket + serial + fifo read lag 0.075 = tested with rviz odom only 
	dt = (now-before).to_sec()
	before = now

	distance = float(s[2])/1000
	delta_x = distance * cos(pos[2])
	delta_y = distance * sin(pos[2]) 
	delta_th = radians(float(s[3]))
	pos[0] += delta_x
	pos[1] += delta_y
	pos[2] += delta_th
	
	# tf
	odom_quat = tf.transformations.quaternion_from_euler(0, 0, pos[2])
	br.sendTransform((pos[0], pos[1], 0), odom_quat, now, "base_link","odom")
	# future
	# quat = tf.transformations.quaternion_from_euler(0, 0, 0)
	# br.sendTransform((-0.054, 0.048, 0.29), quat, now, "camera_depth_frame", "base_link")
	# br.sendTransform((0, 0, 0), quat, now, "odom", "map")
	
	# odom
	odom = Odometry()
	odom.header.stamp = now
	odom.header.frame_id = "odom"

	#set the position
	odom.pose.pose.position.x = pos[0]
	odom.pose.pose.position.y = pos[1]
	odom.pose.pose.position.z = 0.0
	odom.pose.pose.orientation.x = odom_quat[0]
	odom.pose.pose.orientation.y = odom_quat[1]
	odom.pose.pose.orientation.z = odom_quat[2]
	odom.pose.pose.orientation.w = odom_quat[3]
	odom.pose.covariance=  [0.000001, 0, 0, 0, 0, 0, 
							0, 0.000001, 0, 0, 0, 0,    
							0, 0, 0.000001, 0, 0, 0,   
							0, 0, 0, 0.000001, 0, 0, 
							0, 0, 0, 0, 0.000001, 0, 
							0, 0, 0, 0, 0, 0.000001] 

	#set the velocity
	odom.child_frame_id = "base_link"
	odom.twist.twist.linear.x = distance / dt
	odom.twist.twist.linear.y = 0
	odom.twist.twist.linear.z = 0
	odom.twist.twist.angular.x = 0
	odom.twist.twist.angular.y = 0
	odom.twist.twist.angular.z = delta_th / dt
	
	#publish
	odom_pub.publish(odom)

def cleanup():
	oculusprimesocket.sendString("state delete odometrybroadcast")
	oculusprimesocket.sendString("odometrystop")
	# oculusprimesocket.sendString("state delete navigationenabled")
	oculusprimesocket.sendString("log odom_tf.py disconnecting")  # goodbye 



# MAIN

rospy.init_node('odom_tf', anonymous=False)
before = rospy.Time.now()
br = tf.TransformBroadcaster()
odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
rospy.on_shutdown(cleanup)
oculusprimesocket.connect()
oculusprimesocket.sendString("log odom_tf.py connected")  
oculusprimesocket.sendString("state odometrybroadcast 250")  # ms, needs to be set prior to odometrystart
oculusprimesocket.sendString("odometrystart")
broadcast("* * 0 0".split()) # broadcast zero odometry baseline

while not rospy.is_shutdown():

	# t = rospy.get_time()
	# if t-lastupdate > updateinterval:  # request odometry update
		# oculusprimesocket.sendString("odometryreport")

	s = oculusprimesocket.replyBufferSearch("<state> distanceangle ")
	if not s=="":
		broadcast(s.split())
		# lastupdate = now.to_sec()
		
	rospy.sleep(0.005) # was 0.01
