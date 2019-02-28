#!/usr/bin/env python

import rospy, serial, math, os, sys
from sensor_msgs.msg import LaserScan
import oculusprimesocket
import thread


turning = False
READINTERVAL = 0.0014
MINIMUMRANGE = 0.5
RPM = 180 # default 180
SKIPLAST = int(60.0/RPM/READINTERVAL * 0.014)
debugoutput = True
VERSION = "1.0"

def cleanup():
	#  ser.write("p\n") # stop lidar rotation
	#  ser.write("n\n") # disable broadcast
	#  ser.write("0\n") # disable lidar
	ser.write("f\n") # stop lidar
	ser.close()
	removelockfiles()
	rospy.sleep(3)
	print("lidar disabled, shutdown");
	oculusprimesocket.sendString("state lidar false")
	
def directionListenerThread():
	global turning
	while oculusprimesocket.connected:
		s = oculusprimesocket.waitForReplySearch("<state> direction")
		direction = s.split()[2]
		if direction == "left" or direction == "right":
			turning = True
		else:
			turning = False
			
def removelockfiles():
	os.system("rm -f /tmp/dev_ttyUSB*")

def checkBoardId(idstring, ser):
	ser.reset_input_buffer()
	ser.write("x\n") # check board id
	line = ""
	rospy.sleep(0.1)
	while ser.inWaiting() > 0:
		line = ser.readline().strip()

	if not line == idstring:
		rospy.loginfo(rospy.get_name()+" incorrect board id: "+line)
		return False
	
	rospy.loginfo(rospy.get_name()+" connected to: "+line)
	return True
	
def usbdiscover(idstring):
	global lockfilepath
	
	portnum = 0;

	while portnum <= 6 and not rospy.is_shutdown():
		port = '/dev/ttyUSB'+str(portnum)

		rospy.loginfo(rospy.get_name()+" trying port: "+port)
		
		lockfilepath = "/tmp/dev_ttyUSB"+str(portnum)
		tries = 0
		while tries < 5 and not rospy.is_shutdown():
			if os.path.exists(lockfilepath):
				rospy.loginfo(rospy.get_name()+" port busy: "+port)
				rospy.sleep(1)
				tries += 1
			else:
				break 

		if tries == 5:
			rospy.loginfo(rospy.get_name()+" giving up on port: "+port)
			portnum += 1
			continue
			
		open(lockfilepath, 'w') # creates lockfile
		
		try:
			ser = serial.Serial(port, 115200, timeout=5)
		except serial.SerialException: 
			rospy.loginfo(rospy.get_name()+" port exception: "+port)
			os.remove(lockfilepath)
			portnum += 1
			rospy.sleep(1)
			continue
			
		rospy.sleep(2.5)

		if checkBoardId(idstring, ser):
			break
			
		ser.close()
		if os.path.exists(lockfilepath):
			os.remove(lockfilepath)
		rospy.sleep(1)
		portnum += 1
		
	if not ser.is_open:
		rospy.logerr(idstring+" device not found")
		sys.exit(0)
	
	return ser

# main

dropscan = False
scannum = 0

rospy.init_node('xaxxon_lidar', anonymous=False)
rospy.on_shutdown(cleanup)
rospy.loginfo("xaxxon_lidar.py version: "+VERSION)
scan_pub = rospy.Publisher(rospy.get_param('~scan_topic', 'scan'), LaserScan, queue_size=3)

oculusprimesocket.connect()
thread.start_new_thread( directionListenerThread, () )
oculusprimesocket.sendString("state lidar true")

# removelockfiles()
ser = usbdiscover("<id::xaxxonlidar>")

ser.write("y\n") # get version
line = ""
rospy.sleep(0.1)
while ser.inWaiting() > 0:
	line = ser.readline().strip()
	print(line)

""" set speed (180 firmware default if not set here, 255 max) """
ser.write("r"+chr(RPM)+"\n") 

# ser.write("3\n") # disable heartbeat check

# start lidar	
#  ser.write("g\n") # start rotation, full speed
#  ser.write("1\n") # enable lidar
#  ser.write("b\n") # enable broadcast

ser.write("a\n") # start lidar


# clear buffer
ser.reset_input_buffer()

raw_data = []
lastscan = rospy.Time.now()
headercodesize = 4
current_time = 0

while not rospy.is_shutdown() and ser.is_open:
	
	# read data and dump into array, checking for header code 0xFF,0xFF,0xFF,0xFF
	ch = ser.read(1)
	
	if len(ch) == 0:
		rospy.logerr("no response from xaxxonlidar device")
		break
	
	raw_data.append(ch)
	
	if turning:
		dropscan = True
			
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

	ser.write("h\n") # send host hearbeat (every <10 sec minimum)

	# read count		
	low = ord(ser.read(1))
	high = ord(ser.read(1))
	count = (high<<8)|low

	# """ read first distance offset """
	# low = ord(ser.read(1))
	# high = ord(ser.read(1))
	# firstDistanceOffset = ((high<<8)|low)/1000000.0
	
	""" read cycle """
	c1 = ord(ser.read(1))
	c2 = ord(ser.read(1))
	c3 = ord(ser.read(1))
	c4 = ord(ser.read(1))
	cycle = ((c4<<24)|(c3<<16)|(c2<<8)|c1)/1000000.0
	
	# """ read last distance offset """
	# low = ord(ser.read(1))
	# high = ord(ser.read(1))
	# lastDistanceOffset = ((high<<8)|low)/1000000.0
	
	""" device time """
	# if current_time == 0:
		# current_time = rospy.Time.now() # - rospy.Duration(0.0) #0.015
	# else:
		# current_time += rospy.Duration(cycle)
	# rospycycle = current_time - lastscan
	# lastscan = current_time

	""" host time """
	current_time = rospy.Time.now() # - rospy.Duration(0.0) #0.015
	rospycycle = current_time - lastscan
	# cycle = rospycycle.to_sec()
	lastscan = current_time	

	
	rospycount = (len(raw_data)-headercodesize)/2
	
	# startup debug info
	if debugoutput:
		if not count == 0:
			print "cycle: "+str(cycle)
			## print "rospycycle: "+str(rospycycle.to_sec())
			print "count: "+str(count)
			# print "lastDistanceOffset: "+str(lastDistanceOffset)
			## print "firstDistanceOffset: "+str(firstDistanceOffset)
			print "scannum: "+str(scannum)
			# print "interval: "+str(cycle/count)
			## print "raw_data length: "+str((len(raw_data)-headercodesize)/2)
		if not rospycount == count:
			print "*** COUNT/DATA MISMATCH *** "+ str( rospycount-count )
		print " "
	
		if scannum > 20:
			debugoutput = False
	
	scannum += 1	
	if scannum <= 5: # drop 1st few scans while lidar spins up
		del raw_data[:]
		continue
	
	scan = LaserScan()
	scan.header.stamp = current_time - rospycycle # - rospy.Duration(0.01) #rospy.Duration(cycle) 
	scan.header.frame_id = 'laser_frame'

	scan.angle_min = 0
	#  scan.angle_max = (2 * math.pi) 

	# scan.angle_min = (firstDistanceOffset/cycle) * (2 * math.pi)
	# scan.angle_max = (2 * math.pi) - ((lastDistanceOffset/cycle)  * (2 * math.pi))
	
	# scan.angle_increment = READINTERVAL / cycle * 2 * math.pi
	# scan.angle_max = scan.angle_increment * (count-1)
	
	scan.angle_max = (cycle - READINTERVAL*SKIPLAST)/cycle * 2 * math.pi
	# scan.angle_max = (cycle - lastDistanceOffset)/cycle * 2 * math.pi
	scan.angle_increment = scan.angle_max / (count-1)

	#  scan.angle_increment = 2 * math.pi / count
	# scan.angle_increment = (scan.angle_max - scan.angle_min) / (count-1)
	scan.time_increment =  cycle/count
	scan.scan_time = cycle # rospycycle.to_sec()
	scan.range_min = 0.05
	scan.range_max = 20.0

	# temp = []
	zeroes = 0
	scan.ranges=[]
	for x in range(len(raw_data)-(count*2)-headercodesize, len(raw_data)-headercodesize, 2):
		low = ord(raw_data[x])
		high = ord(raw_data[x+1])
		value = ((high<<8)|low) / 100.0
		#  print value
		if value < MINIMUMRANGE:
			value = 0
		scan.ranges.append(value)
		

	""" comp rpm photo sensor offset """
	# tilt = 283 # degrees  (was 280)
	# split = int(tilt/360.0*count)
	# scan.ranges = temp[split:]+temp[0:split]

	""" masking frame """
	# maskwidth = 8 # half width, degrees
	# masks = [90,133, 270, 315] # proto 2 
	# for m in masks:
		# for x in range(int(count*((m-maskwidth)/360.0)), int(count*((m+maskwidth)/360.0)) ):
			# scan.ranges[x] = 0
	
	#  masks = [90, 145, 275, 330]
	#masks = [43, 54,    78, 104,    130, 147,   263, 286,    312, 329]	   #223, 134,   	
	masks = [35,50,78,102,130,144,215,230,258,282,310,324]
	#  masks = []
	
	i = 0
	offst = 2 # slight mask rotation comp
	while i < len(masks):
		for x in range(int(count*((masks[i]+offst)/360.0)), int(count*((masks[i+1]+offst)/360.0)) ):
			scan.ranges[x] = 0
		i += 2
		
	#  i = 0	
	#  zeroed = 0
	#  spread = 2
	#  while i < len(scan.ranges):
		#  if scan.ranges[i] == 0.04: # blank point found
			#  for n in range(spread*2+1):
				#  m = i+(n-spread)
				#  if m >=0 and m<len(scan.ranges):
					#  scan.ranges[m] = 0
					#  zeroed += 1
			#  i += spread
		#  elif scan.ranges[i] < MINIMUMRANGE:
			#  scan.ranges[i] = 0
		#  i += 1
	
	#  print zeroed				 
	
				
	if dropscan: 	# blank scans when turning
		for i in range(len(scan.ranges)):
			scan.ranges[i] = 0
	dropscan = False
		
	scan_pub.publish(scan)
	
	del raw_data[:] 

	# if scannum % 10 == 0:
		# msg = "scan #: "+str(scannum)
		# rospy.loginfo(msg)

