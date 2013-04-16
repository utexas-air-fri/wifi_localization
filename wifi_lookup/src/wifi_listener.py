#!/usr/bin/env python

import roslib; roslib.load_manifest('wifi_lookup')
import rospy, os, re, pickle
from wifi_lookup.msg import WifiData, Wifi

#1st goal, Completed, create a method to build a serialized database which is shared between
#	different call-time instances of the node
#2nd goal, redefine the database to perform the two-dimensional hash lookup
#3rd goal, create member method which publishes location based on lookup and input message
dbLoc = "database.pk"

def output(data):
	for spot in data.HotSpots:
		print spot.MAC, spot.dB
		database[spot.MAC] = spot.dB
#create proper handler function to append new hotspots to data object

#serialize the data for storave
def clean():
	dbFile = open(dbLoc,"w")
	pickle.dump(database, dbFile)
	dbFile.close()

#deserialize the object and do ROS things
def make():
	global database
	try:
		dbFile = open(dbLoc)
		database = pickle.load(dbFile)
		dbFile.close()
	except: 
		database = {}
	rospy.Subscriber('wifi_data', WifiData, output)
	rospy.spin()

if __name__=='__main__':
	rospy.init_node('wifi_listener')
	try:
		make()
	except rospy.ROSInterruptException: pass
	clean()
