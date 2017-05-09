#!/usr/bin/env python

import oculusprimesocket, math, time

oculusprimesocket.connect()
# oculusprimesocket.sendString("odometrystop")
# time.sleep(0.5)
# oculusprimesocket.sendString("state odometrybroadcast 250")  # ms
# oculusprimesocket.sendString("odometrystart")

while True:
	s = oculusprimesocket.waitForReplySearch("<state> distanceangle ")
	s = s.split()
	distance = float(s[2]) / 1000 # meters
	angle = float(s[3]) # degrees

	if not distance == 0:
		turnrate = angle/distance
		print (str(distance)+"m "+str(angle)+"deg "+str(turnrate)+"dpm")

"""
typical output for malgcommand f 100 255, then stop command  4 sec total (observed goes RIGHT)
0.043m 2.5833deg 60.076744186dpm << discard, only portion of broadcast period, results wacky
0.086m 2.6596deg 30.9255813953dpm << mega initial drift  
0.089m 0.7521deg 8.45056179775dpm << accel?
0.089m 1.0028deg 11.2674157303dpm
0.089m 0.9701deg 10.9dpm
0.089m 0.8611deg 9.67528089888dpm
0.093m 0.8393deg 9.0247311828dpm
0.089m 1.0573deg 11.8797752809dpm
0.089m 1.0137deg 11.3898876404dpm
0.089m 0.8829deg 9.9202247191dpm
0.089m 0.8393deg 9.43033707865dpm
0.093m 0.9701deg 10.4311827957dpm
0.089m 0.9919deg 11.1449438202dpm
0.089m 0.7085deg 7.9606741573dpm << deaccel
0.063m 0.3924deg 6.22857142857dpm << deaccel
0.019m 0.0545deg 2.86842105263dpm << discard, only portion of broadcast period, results wacky

other way f 255 100 (observed goes RIGHT faster turn than left)
0.009m -0.0545deg -6.05555555556dpm
0.053m -4.5126deg -85.1433962264dpm
0.086m -2.2236deg -25.8558139535dpm
0.083m -2.1255deg -25.6084337349dpm
0.086m -1.9402deg -22.5604651163dpm
0.086m -1.9402deg -22.5604651163dpm
0.086m -1.8966deg -22.0534883721dpm
0.086m -2.0383deg -23.7011627907dpm
0.086m -1.9075deg -22.1802325581dpm
0.086m -1.7549deg -20.4058139535dpm
0.083m -2.0165deg -24.2951807229dpm
0.086m -2.0928deg -24.3348837209dpm
0.086m -1.6895deg -19.6453488372dpm
0.086m -1.8094deg -21.0395348837dpm
0.086m -2.0819deg -24.2081395349dpm
0.086m -1.9511deg -22.6872093023dpm
0.086m -1.7004deg -19.7720930233dpm
0.086m -2.0492deg -23.8279069767dpm
0.069m -1.1772deg -17.0608695652dpm
0.026m 0.327deg 12.5769230769dpm

f 30 255
0.023m 0.0218deg 0.947826086957dpm
0.069m 9.5375deg 138.224637681dpm
0.089m 8.5238deg 95.7730337079dpm
0.089m 6.1694deg 69.3191011236dpm
0.086m 5.3955deg 62.738372093dpm
0.086m 4.6652deg 54.2465116279dpm
0.086m 4.6216deg 53.7395348837dpm
0.089m 4.6434deg 52.1730337079dpm
0.089m 4.905deg 55.1123595506dpm
0.086m 4.6107deg 53.6127906977dpm
0.089m 4.6761deg 52.5404494382dpm
0.086m 4.7524deg 55.2604651163dpm
0.089m 4.7197deg 53.0303370787dpm
0.089m 4.796deg 53.8876404494dpm
0.043m 1.3189deg 30.6720930233dpm
0.003m 0.0deg 0.0dpm

f 255 30
0.003m -0.0218deg -7.26666666667dpm
0.023m -10.7256deg -466.330434783dpm
0.046m -9.8318deg -213.734782609dpm
0.056m -7.1831deg -128.269642857dpm
0.059m -6.2675deg -106.228813559dpm
0.063m -5.3955deg -85.6428571429dpm
0.063m -5.3192deg -84.4317460317dpm
0.063m -5.5372deg -87.8920634921dpm
0.059m -5.6244deg -95.3288135593dpm
0.063m -5.0685deg -80.4523809524dpm
0.063m -5.45deg -86.5079365079dpm
0.063m -5.5372deg -87.8920634921dpm
0.063m -5.4173deg -85.9888888889dpm
0.059m -5.4173deg -91.8186440678dpm
0.053m -3.9022deg -73.6264150943dpm
0.013m 0.436deg 33.5384615385dpm
"""
