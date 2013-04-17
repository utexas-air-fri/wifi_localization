#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"
#include "ardrone_autonomy/Navdata.h"
#include "ardrone_autonomy/FlightAnim.h"
#include <string>

#define _USE_MATH_DEFINES
#include "math.h"

ros::Publisher velocity_pub;
ros::Publisher land_pub;
ros::Publisher takeoff_pub;
ros::Publisher reset_pub;
ros::Publisher led_pub;

std::string msg = 
	"Control the ardrone.\n" 
	"Hold the WiiMote horizontally.\n"
	"Takeoff: [+]\t Land: [-]\n"
	"Hover: [A]\n"
	"Pitch/ Roll: tilt controls\n"
	"Yaw: [1]/[2]\n"
	"Altitude: [up]/[down]";

// Global variables for Absolute Control
bool firstZ = false;
bool Zlock = false;
int currentZ = 0;
int lockedZ = 0;

int mode = 0;

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg){

	geometry_msgs::Twist output;
	std_msgs::Empty empty;
	
	// Takeoff/ Land/ Reset toggles	
	if(msg->buttons[4] == 1) {
		firstZ = false;
		Zlock = false;
		currentZ = 0;
		lockedZ = 0;
		mode = 0;
		takeoff_pub.publish(empty);
	}
	if(msg->buttons[5] == 1)
		land_pub.publish(empty);
	if(msg->buttons[10] == 1)
		reset_pub.publish(empty);

	// Mode toggles
	if(msg->buttons[9] == 1)
		mode = 0;
	if(msg->buttons[8] == 1)
		mode = 0; // CHANGE TO 1 TO RE-ENABLE ABSOLUTE CONTROL

	// Relative control	
	if(mode == 0) {
		// Pitch	
		output.linear.x	= (msg->axes[0]/10);
		// Roll		
		output.linear.y = (msg->axes[1]/10);
	}

	// Absolute control
	// TODO: FIX THIS
	else if(mode == 1) {
		if(!Zlock) {
			lockedZ = currentZ;
			Zlock = true;
		}
		if(msg->buttons[3] == 1 || !firstZ)
			Zlock = false;

		double tranX = cos((lockedZ - currentZ)*M_PI/180.0);
		double tranY = sin((lockedZ - currentZ)*M_PI/180.0);
		output.linear.x = tranX*(msg->axes[0]/10);
		output.linear.y = tranY*(msg->axes[0]/10);
	}

	// Yaw
	if(msg->buttons[0] == 1 && msg->buttons[1] == 0)
		output.angular.z = .75;
	else if(msg->buttons[0] == 0 && msg->buttons[1] == 1)
		output.angular.z = -.75;

	// Altitude control
	if(msg->buttons[6] == 1 && msg->buttons[7] == 0)
		output.linear.z = -1;
	else if(msg->buttons[6] == 0 && msg->buttons[7] == 1)
			output.linear.z = 1;
	
	// If hover is on, override all axes to 0
	if(msg->buttons[2] == 1) {
		output.linear.x = 0;
		output.linear.y = 0;
		output.angular.z = 0;
		
		// Flip Forward
		if(msg->buttons[7] == 1) {
			ardrone_autonomy::FlightAnim animate;
			animate.request.type = 16;
			animate.request.duration = 0;
			ros::service::call("ardrone/setflightanimation", animate);
		}
		// Flip Back
		if(msg->buttons[6] == 1) {
			ardrone_autonomy::FlightAnim animate;
			animate.request.type = 17;
			animate.request.duration = 0;
			ros::service::call("ardrone/setflightanimation", animate);
		} 
		// Flip Left
		if(msg->buttons[8] == 1) {
			ardrone_autonomy::FlightAnim animate;
			animate.request.type = 18;
			animate.request.duration = 0;
			ros::service::call("ardrone/setflightanimation", animate);
		} 
		// Flip Right
		if(msg->buttons[9] == 1) {
			ardrone_autonomy::FlightAnim animate;
			animate.request.type = 19;
			animate.request.duration = 0;
			ros::service::call("ardrone/setflightanimation", animate);
		} 
	}
	
	velocity_pub.publish(output);
}

void navCallback(const ardrone_autonomy::Navdata::ConstPtr& msg){
	// Get the Z of the quadcopter
	firstZ = true;
	currentZ = (((int)msg->rotZ)+360)%360;
	
	// Get battery level
	std::cout << "Battery Level: " << msg->batteryPercent << std::endl;
}

int main(int argc, char **argv){

	std::cout << msg << std::endl;

	ros::init(argc, argv, "follower");
	ros::NodeHandle n;

	// Velocity publisher
	velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	// Takeoff/ land/ reset publishers
	land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 1000);
	takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1000);
	reset_pub = n.advertise<std_msgs::Empty>("ardrone/reset", 1000);

	ros::Subscriber sub1 = n.subscribe("ardrone/navdata", 1000, navCallback); 	
	ros::Subscriber sub2 = n.subscribe("joy", 1000, joyCallback);   

	ros::Rate loop_rate(10);

	ros::spin();

	return 0;
}
