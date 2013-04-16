#!/usr/bin/env python

import roslib; roslib.load_manifest('wifi_lookup')
import rospy, pickle
from wifi_lookup.msg import WifiData, Wifi

#1st goal, Completed, create a method to build a serialized database which is shared between
#	different call-time instances of the node
#2nd goal, Completed, redefine the database to perform the two-dimensional hash lookup
#3rd goal, modify the listener for x,y injection
#	make sure that the location injection prevents duplicates, it currently doesn't check
dbLoc = "database.pk"

#I'm not sure if all of the layers of temps and resetting are needed, but they are safe
def inject(data):
	for spot in data.HotSpots:
		print spot.MAC, spot.dB
		if spot.MAC in database:
			firstTemp = database[spot.MAC]
		else:
			firstTemp = {}
		if spot.dB in firstTemp:
			secondTemp = firstTemp[spot.dB]
		else:
			secondTemp = []
		secondTemp.append((0,0)) #currently bad
		firstTemp[spot.dB] = secondTemp
		database[spot.MAC] = firstTemp
	
#serialize the data for storage
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
	rospy.Subscriber('wifi_data', WifiData, inject)
	rospy.spin()

if __name__=='__main__':
	rospy.init_node('wifi_listener')
	try:
		make()
	except rospy.ROSInterruptException: pass
	clean()
