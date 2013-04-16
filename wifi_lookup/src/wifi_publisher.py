#!/usr/bin/env python

import roslib; roslib.load_manifest('wifi_lookup')
import rospy, os, re, pickle
from wifi_lookup.msg import WifiData, Wifi

#1st goal, Completed, create a method to build a serialized database which is shared between
#	different call-time instances of the node
#2nd goal, redefine the database to perform the two-dimensional hash lookup
#3rd goal, create member method which publishes location based on lookup and input message
dbLoc = "database.pk"

#create proper handler function to append new hotspots to data object

#deserialize the object and do ROS things
def make():
	global database
	try:
		dbFile = open(dbLoc)
		database = pickle.load(dbFile)
		dbFile.close()
	except: 
		database = {}

if __name__=='__main__':
	rospy.init_node('wifi_publisher')
	try:
		make()
	except rospy.ROSInterruptException: pass
