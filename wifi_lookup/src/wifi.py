#!/usr/bin/env python

import roslib; roslib.load_manifest('wifi_lookup')
import rospy, os, re
from wifi_lookup.msg import WifiData, StrArr

class Node():
	def __init__(self):
		pub = rospy.Publisher('wifi_data', WifiData)

		r = rospy.Rate(1)
		while not rospy.is_shutdown():
			os.system("iwlist wlan0 scanning >> datatemp.txt")

			wifiraw = open("datatemp.txt").read()
			os.remove("datatemp.txt")

			essids = re.findall("ESSID:\"(.*)\"", wifiraw)
			addresses = re.findall("Address: ([0-9A-F:]{17})", wifiraw)
			signals = re.findall("Signal level=.*?([0-9]+)", wifiraw)

			msg = WifiData()

			for i in range(len(essids)):
				if (essids[i] == "restricted.utexas.edu"):
					temp = StrArr()			    
					temp.MAC = addresses[i] 
					temp.dB = signals[i]
					msg.HotSpots.append(temp)

			pub.publish(msg)
			r.sleep()

if __name__ == '__main__':
	rospy.init_node('wifi_lookup')
	try:
		n = Node()
	except rospy.ROSInterruptException: pass
	
