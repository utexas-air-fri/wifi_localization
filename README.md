Freshman Research Initative - Building Wide Intelligence Code 2013
==================

This repository is where Robert and Josh are storing the code written for their projects in the BWI lab.

Wifi Lookup
-----------

ROS based WiFi location code (currently under development)

This node consists of three parts to accomplish localization. They include:
- [x] WiFi_Data: Get dB and MAC addresses for all visible WiFi access points with the ESSID provided as a parameter.
- [x] WiFi_Listener: Takes the data from WiFi data and saves using serialization in a Data Structure for lookup later.
- [x] WiFi_Publisher: Reads data serialized from the Listener and publishes it in (X,Y) coords based off the origin provided as a parameter.

Check the wiki for more information on the project:
[Class Wiki](http://farnsworth.csres.utexas.edu/bwi/index.php/CS378/WiFi_Localization)
