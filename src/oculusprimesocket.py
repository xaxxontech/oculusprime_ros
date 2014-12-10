#!/usr/bin/env python

# socketclient.py - make tcp socket connection with robot, relay commands and messages 


import socket, re

# NETWORK VARIABLES - change to appropriate values
host = "127.0.0.1"
port = 4444

connected = True
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 

# send single line commands to java server
def sendString(s):
	try:
		clearIncoming()
		sock.sendall(s+"\r\n")
	except socket.error: 
		connected = False

# [blocking] search output from java server, WAIT until match to regular expresson pattern
def waitForReplySearch(pattern): 
	while True:
		try:
			servermsg = (sockfileIO.readline()).strip()
			#print("script recieved: "+servermsg)
			if re.search(pattern, servermsg, re.IGNORECASE): 
				break
		except socket.error: 
			connected = False
	return servermsg # return the line containing pattern

def clearIncoming():
	sock.setblocking(False)
	while True:
		try:
			sockfileIO.readline()
		except socket.error:
			break
	sock.setblocking(True)
			
# [non-blocking] search through unread output from java server, compare to regular expresson pattern
def replyBufferSearch(pattern): 
	result = "" # return empty string if search fails
	sock.setblocking(False) # don't pause and wait for any further input
	while True:
		try:
			servermsg = (sockfileIO.readline()).strip()
			#print(servermsg)
			if re.search(pattern, servermsg, re.IGNORECASE): 
				result = servermsg # return the line containing pattern
				break
		except socket.error: # assuming EOF reached, reading buffer complete
			break
	sock.setblocking(True)
	return result  


# connect
try:
	sock.connect((host, port))
except socket.error:
	connected = False
sockfileIO = sock.makefile()

# login 	
waitForReplySearch("^<telnet> Welcome")
