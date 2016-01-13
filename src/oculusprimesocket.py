# oculusprimesocket.py module
# make tcp socket connection with Oculus Prime robot, relay commands and messages 

"""make tcp socket connection with Oculus Prime Server Application
provide functions for relay of commands and messages"""

import socket, re, time

# NETWORK VARIABLE defaults - change to appropriate values
host = "127.0.0.1"
port = 4444

connected = False
reconnect = False

sock = None
sockfileIO = None


def sendString(s):
	"""Send single line command to server
	
	s -- command to be sent
	"""

	global connected
	try:
		sock.sendall(s+"\r\n")
	except socket.error: 
		connected = False
		if reconnect:
			waitForConnect()
			sendString(s)


def waitForReplySearch(pattern):
	"""Read all incoming messages from server, do not return until search match 
	
	pattern -- regular expression pattern to be searched
	returns first line containing match, or empty string if server shutdown
	blocking function
	"""
	
	global connected
	while True:
		try:
			servermsg = (sockfileIO.readline()).strip()

			if re.search("<telnet> shutdown", servermsg, re.IGNORECASE):
				connected = False
				if reconnect:
					waitForConnect()
					waitForReplySearch(pattern)
					return
				else:
					return ""		
					
			if re.search(pattern, servermsg, re.IGNORECASE): 
				break
		except socket.error: 
			connected = False
			return ""
	return servermsg # return the line containing pattern


def clearIncoming():
	"""Clear socket buffer of all incoming server messages"""

	sock.setblocking(False)
	while True:
		try:
			sockfileIO.readline()
		except socket.error: # assuming EOF reached, reading buffer complete
			break
			
	sock.setblocking(True)
			
			
def replyBufferSearch(pattern): 
	"""Search through unread output from server, compare to pattern, return match
	stops reading buffer if finds a match
	
	pattern -- regular expression pattern to be searched
	returns: first line containing match, or empty string if search fails
	non blocking function
	"""
	
	# global connected
	result = "" # return empty string if search fails
	sock.setblocking(False) # don't pause and wait for any further input
	while True:
		try:
			servermsg = (sockfileIO.readline()).strip()
			if re.search(pattern, servermsg, re.IGNORECASE): 
				result = servermsg # return the line containing pattern
				break
		except socket.error: # assuming EOF reached, reading buffer complete
			break
	sock.setblocking(True)
	return result  


def connect():
	"""Make socket connection to server, blocking
	
	returns: True if success, False otherwise
	"""
	global sockfileIO, connected, sock
	connected = False
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
	sockfileIO = None
	
	try:
		sock.connect((host, port))
	except socket.error:
		connected = False
		if reconnect:
			waitForConnect()
			return True
		else: 
			return False
	sockfileIO = sock.makefile()
	waitForReplySearch("^<telnet> Welcome")
	connected = True
	return True
	
	
def waitForConnect():
	while not connected:
		time.sleep(10)
		connect()
