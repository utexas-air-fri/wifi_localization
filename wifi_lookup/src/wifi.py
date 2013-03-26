#!/usr/bin/env python

import roslib; roslib.load_manifest('wifi_lookup')
import rospy, os, re
from std_msgs.msg import String

pub = rospy.Publisher('wifi_data', String)
rospy.init_node('wifi_lookup')

r = rospy.Rate(1)
while not rospy.is_shutdown():
    os.system("iwlist wlan0 scanning >> datatemp.txt")

    wifiraw = open("datatemp.txt").read()
    os.remove("datatemp.txt")

    # Sorry not sorry
    essids = re.findall("ESSID:\"(.*)\"", wifiraw)
    addresses = re.findall("Address: ([0-9A-F:]{17})", wifiraw)
    signals = re.findall("Signal level=.*?([0-9]+)", wifiraw)
    spots = ["MAC: " + addresses[i] + "\nRATE: " + signals[i] for i in range(len(essids)) if essids[i] =="restricted.utexas.edu"]

    pub.publish("\n".join(spots))
    r.sleep()

