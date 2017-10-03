#!/usr/bin/env python

import rospy, serial, math
from sensor_msgs.msg import LaserScan


def cleanup():
	ser.write("p\n"); # stop lidar rotation
	ser.write("n\n"); # disable broadcast
	ser.write("0\n"); # disable lidar
	ser.close();


# main

rospy.init_node('xaxxon_lidar', anonymous=False)
rospy.on_shutdown(cleanup)
scan_pub = rospy.Publisher('scan', LaserScan, queue_size=10)


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
	ser.close()
else:	
	# start lidar	
	ser.write("1\n") # enable lidar
	ser.write("b\n") # enable broadcast
	ser.write("g\n"); # start rotation
	# ser.write("m")
	# ser.write(chr(99))
	# ser.write("\n")

	# clear buffer
	ser.reset_input_buffer()

raw_data = []
scannum = 0

while not rospy.is_shutdown() and ser.is_open:
	# read data and dump into array, checking for header code 0xFF,0xFF
	ch = ser.read(1)
	raw_data.append(ch)
	if not ord(ch) == 0xFF:
		continue
	else:
		ch = ser.read(1)
		raw_data.append(ch)
		if not ord(ch) == 0xFF:
			continue
		
	# read header count		
	low = ord(ser.read(1))
	high = ord(ser.read(1))
	count = (high<<8)|low
	
	#read header cycle
	low = ord(ser.read(1))
	high = ord(ser.read(1))
	cycle = (high<<8)|low

	print "cycle: "+str(cycle)
	print "count: "+str(count)
	print "raw_data length: "+str(len(raw_data))
	
	scannum += 1	
	if scannum < 8: # drop 1st few scans while lidar spins up
		del raw_data[:]
		continue
	
	current_time = rospy.Time.now()
	scan = LaserScan()

	scan.header.stamp = current_time
	scan.header.frame_id = 'laser_frame'
	scan.angle_min = 0.0
	scan.angle_max = 2 * math.pi
	scan.angle_increment = 2 * math.pi / count
	scan.time_increment = cycle/count/1000.0
	scan.range_min = 0.1
	scan.range_max = 20.0

	# scan.ranges[0:count-1] = raw_data[len(raw_data)-count-3:len(raw_data)-3]
	scan.ranges = []
	for x in range(len(raw_data)-(count*2)-2, len(raw_data)-2, 2):
		low = ord(raw_data[x])
		high = ord(raw_data[x+1])
		scan.ranges.append(((high<<8)|low) / 100.0)

	scan_pub.publish(scan)

	del raw_data[:] 

