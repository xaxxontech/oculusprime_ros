#!/usr/bin/env python
import rospy
import oculusprime
from sensor_msgs.msg import Joy

lastdirection = "S" # F B L R S (forward back left right stop)
lastcam = "S" # U D S (up down stop)
laststrobe = "off"
lastdock = "nil"

# controls:
# left stick: move 
# X - strobe blast
# (triangle) - press once with dock in camera view to autodock 

#to run:
#sudo bash
#rosrun ps3joy 
#roslaunch oculusprime oculussd3.launch 

# ds3joy broadcasting on /joy
# msg type: sensor_msgs/Joy:
	#std_msgs/Header header
	  #uint32 seq
	  #time stamp
	  #string frame_id
	#float32[] axes
	#int32[] buttons
  
def callback(data):
	
	## determine controller input
	th = 0.6 #threshold
	
	direction = "S"
	horiz = data.axes[0]
	vert = data.axes[1]
	if abs(horiz) < th and abs(vert) < th:
		direction = "S"
	elif abs(horiz) > abs(vert): #horiz
		if horiz < 0:
			direction = "R"
		else:
			direction = "L"
	else:						  #vert
		if vert > 0:
			direction = "F"
		else:
			direction = "B"

	cam = "S"
	cvert = data.axes[3]
	if abs(cvert) > th:
		if cvert < 0:
			cam = "U"
		else:
			cam = "D"
			
	strobebutton = data.buttons[14]
	strobe = "off"
	if strobebutton == 1:
		strobe = "on"
		
	dockbutton = data.buttons[12]
	dock = "nil"
	if dockbutton == 1:
		dock = "go"
	
	## send orders to bot
	
	# move
	global lastdirection
	if direction == "S" and lastdirection != "S":
		oculusprime.sendString("move stop")
		lastdirection = direction
	if direction == "F" and lastdirection != "F":
		oculusprime.sendString("move forward")
		lastdirection = direction
	elif direction == "B" and lastdirection != "B":
		oculusprime.sendString("move backward")
		lastdirection = direction
	elif direction == "R" and lastdirection != "R":
		oculusprime.sendString("move right")
		lastdirection = direction
	elif direction == "L" and lastdirection != "L":
		oculusprime.sendString("move left")
		lastdirection = direction
		
	# camera tilt
	global lastcam
	if cam == "S" and lastcam != "S":
		oculusprime.sendString("cameracommand stop")
		lastcam = cam
	elif cam == "U" and lastcam != "U":
		oculusprime.sendString("cameracommand up")
		lastcam = cam
	elif cam == "D" and lastcam != "D":
		oculusprime.sendString("cameracommand down")
		lastcam = cam
		
	# strobe
	global laststrobe
	if strobe == "on" and laststrobe != "on":
		oculusprime.sendString("strobeflash on")
		laststrobe = strobe
	elif strobe == "off" and laststrobe != "off":
		oculusprime.sendString("strobeflash off")
		laststrobe = strobe
		
	# dock
	global lastdock
	if dock=="go" and lastdock != "go":
		oculusprime.sendString("streamsettingsset high");
		oculusprime.sendString("publish camera")
		rospy.sleep(2)
		oculusprime.sendString("autodock go")
		lastdock = dock
	elif dock=="nil":
		lastdock = "nil"

				
	#print("L/R: "+str(data.axes[0])+"       U/D: "+str(data.axes[1]))
	

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("joy", Joy, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()


