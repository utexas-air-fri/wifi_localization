#!/usr/bin/env python

import roslib; roslib.load_manifest('wifi_lookup')
import rospy, os, re, pickle
from wifi_lookup.msg import WifiData, Wifi

#1st goal, create a method to build a serialized database which is shared between
#	different call-time instances of the node
#2nd goal, redefine the database to perform the two-dimensional hash lookup
#3rd goal, create member method which publishes location based on lookup and input message
class ListenNode():
	#example handler function
	def output(data):
		print "I heard ", data.length, " Hotspots"

	#create proper handler function to append new hotspots to data object

	def __init__(self):
	#de-serialize the data object	
		rospy.Subscriber('wifi_data', WifiData, output)
		rospy.spin()
	#create routine to serialize the data object on shutdown

if __name__=='__main__':
	rospy.init_node('wifi_listener')
	try:
		n = ListenNode()
	except rospy.ROSInterruptException: pass
