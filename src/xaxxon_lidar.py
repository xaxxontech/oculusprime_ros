#!/usr/bin/env python

import rospy, serial, math
from sensor_msgs.msg import LaserScan


def cleanup():
	ser.write("p\n"); # stop lidar rotation
	ser.write("n\n"); # disable broadcast
	ser.write("0\n"); # disable lidar
	ser.close();
	print("lidar disabled, shutdown");

# main

rospy.init_node('xaxxon_lidar', anonymous=False)
rospy.on_shutdown(cleanup)
scan_pub = rospy.Publisher('scan', LaserScan, queue_size=3)


# connect
ser = serial.Serial('/dev/ttyUSB0', 115200)
rospy.sleep(2)

ser.write("x\n") # check board id
line = ""
rospy.sleep(0.1)
while ser.inWaiting() > 0:
	line = ser.readline().strip()
	print(line)

if not line == "<id::xaxxonlidar>":
	rospy.signal_shutdown("incorrect board id")
	print("incorrect board id")
	ser.close()
else:	
	ser.write("y\n") # get version
	line = ""
	rospy.sleep(0.1)
	while ser.inWaiting() > 0:
		line = ser.readline().strip()
		print(line)

	# start lidar	
	ser.write("1\n") # enable lidar
	ser.write("b\n") # enable broadcast
	
	ser.write("g\n") # start rotation, full speed
	
	""" slower speed option: """
	# ser.write("m")
	# ser.write(chr(79))
	# ser.write("\n")

	# clear buffer
	ser.reset_input_buffer()

raw_data = []
scannum = 0
lastscan = rospy.Time.now()
headercodesize = 4

while not rospy.is_shutdown() and ser.is_open:
	
	# read data and dump into array, checking for header code 0xFF,0xFF,0xFF,0xFF
	ch = ser.read(1)
	raw_data.append(ch)
	if not ord(ch) == 0xFF:
		continue
	else:
		ch = ser.read(1)
		raw_data.append(ch)
		if not ord(ch) == 0xFF:
			continue
		else: 
			ch = ser.read(1)
			raw_data.append(ch)
			if not ord(ch) == 0xFF:
				continue
			else: 
				ch = ser.read(1)
				raw_data.append(ch)
				if not ord(ch) == 0xFF:
					continue

	# read count		
	low = ord(ser.read(1))
	high = ord(ser.read(1))
	count = (high<<8)|low

	"""
	# read last distance offset
	low = ord(ser.read(1))
	high = ord(ser.read(1))
	lastDistanceOffset = ((high<<8)|low)/1000000.0

	# read first distance offset
	low = ord(ser.read(1))
	high = ord(ser.read(1))
	firstDistanceOffset = ((high<<8)|low)/1000000.0
	
	# read cycle
	c1 = ord(ser.read(1))
	c2 = ord(ser.read(1))
	c3 = ord(ser.read(1))
	c4 = ord(ser.read(1))
	cycle = ((c4<<24)|(c3<<16)|(c2<<8)|c1)/1000000.0
	"""
	
	current_time = rospy.Time.now() - rospy.Duration(0.015)
	rospycycle = current_time - lastscan
	cycle = rospycycle.to_sec()
	lastscan = current_time
	
	rospycount = (len(raw_data)-headercodesize)/2
	
	# """
	if not count == 0:
		print "cycle: "+str(cycle)
		print "rospycycle: "+str(rospycycle.to_sec())
		print "count: "+str(count)
		# print "lastDistanceOffset: "+str(lastDistanceOffset)
		# print "firstDistanceOffset: "+str(firstDistanceOffset)
		print "scannum: "+str(scannum)
		print "interval: "+str(cycle/count)
		print "raw_data length: "+str((len(raw_data)-headercodesize)/2)
	if not rospycount == count:
		print "*** COUNT/DATA MISMATCH *** "+ str( rospycount-count )
	print " "

	# """
	
	# count = rospycount
	
	scannum += 1	
	if scannum <= 10: # drop 1st few scans while lidar spins up
		del raw_data[:]
		continue

	scan = LaserScan()
	scan.header.stamp = current_time - rospycycle - rospy.Duration(0.01) #rospy.Duration(cycle) 
	scan.header.frame_id = 'laser_frame'

	scan.angle_min = 0
	scan.angle_max = (2 * math.pi) 

	# scan.angle_min = (firstDistanceOffset/cycle) * (2 * math.pi)
	# scan.angle_max = (2 * math.pi) - ((lastDistanceOffset/cycle)  * (2 * math.pi))

	scan.angle_increment = 2 * math.pi / count
	# scan.angle_increment = (scan.angle_max - scan.angle_min) / (count-1)
	scan.time_increment =  cycle/count
	scan.scan_time = rospycycle.to_sec()
	scan.range_min = 0.05
	scan.range_max = 20.0

	temp = []
	for x in range(len(raw_data)-(count*2)-headercodesize, len(raw_data)-headercodesize, 2):
		low = ord(raw_data[x])
		high = ord(raw_data[x+1])
		temp.append(((high<<8)|low) / 100.0)

	# mitigate rpm sensor offset
	tilt = 10 # degrees
	split = int(tilt/360.0*count)
	scan.ranges = temp[split:]+temp[0:split]

	#masking frame
	maskwidth = 8 # half width, degrees
	masks = [265, 295, 97, 126]
	for m in masks:
		for x in range(int(count*((m-maskwidth)/360.0)), int(count*((m+maskwidth)/360.0)) ):
			scan.ranges[x] = 0
		
		
	scan_pub.publish(scan)
	
	del raw_data[:] 


