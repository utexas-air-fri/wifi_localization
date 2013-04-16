Freshman Research Initative - Building Wide Intelligence Code 2013
==================

This repository is where Robert and Josh are storing the code written for their projects in the BWI lab.

ARDrone Fly
-----------

ROS based code used to fly the ARDrone 2 Quadcopter in various ways. The general dependency for this package is the [ARDrone_Autonomy](https://github.com/AutonomyLab/ardrone_autonomy) package.  
The wiimote-based nodes require ROS [wiimote drivers](https://github.com/ros-drivers/joystick_drivers).
To run this node:
* Start the ARDrone drivers (ardrone_autonomy ardrone_driver)
* Start the willmote drivers (wiimote wiimote_node.py)
* Start the controller code

The Ball following node requires a blob tracking driver. For our purposes, [CMVision](https://github.com/dutchcheesehead/ROSMAV/tree/master/cmvision) was used. The colors.txt file in the root of the package contains the data for which blob it will follow.
To run this node:
* Start the ARDrone drivers (ardrone_autonomy ardrone_driver)
* Start CMVision (ardrone_fly cmvision_blob_detector.launch)
* Start the controller code

Wifi Lookup
-----------

ROS based WiFi location code (currently under development)

This node consists of three parts to accomplish localization. They include:
- [x] WiFi_Data: Get dB and MAC addresses for all visible WiFi access points with the ESSID provided as a parameter.
- [ ] WiFi_Listener: Takes the data from WiFi data and saves using serialization in a Data Structure for lookup later.
- [ ] WiFi_Publisher: Reads data serialized from the Listener and publishes it in (X,Y) coords based off the origin provided as a parameter.

Check the wiki for more information on the project:
[Class Wiki](http://farnsworth.csres.utexas.edu/bwi/index.php/CS378/WiFi_Localization)
