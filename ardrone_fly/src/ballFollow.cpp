#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "cmvision/Blobs.h"

ros::Publisher velocity_pub;

void blobCallback(const cmvision::Blobs::ConstPtr& msg){

	geometry_msgs::Twist output;

	if (msg->blob_count > 0){

		double largeArea = 1000, centerX = 320, centerY = 180;

		for (int i = 0; i < msg->blob_count; i++){
					
			if(msg->blobs[i].area > largeArea) {
				largeArea = msg->blobs[i].area;
				centerX = msg->blobs[i].x;
				centerY = msg->blobs[i].y;
			}
		}

		std::cout << "Blob area " << largeArea << " x: " << centerX << " y: " << centerY << std::endl;
		
		output.linear.z = (180 - centerY)/180;
		output.angular.z = (320 - centerX)/320;
		if(largeArea > 10000)
			output.linear.x = -.3;
		else
			output.linear.x = (largeArea > 500) ? (10000 - largeArea)/250000 : 0;
		velocity_pub.publish(output);
	}
}


int main(int argc, char **argv){

	ros::init(argc, argv, "follower");
	ros::NodeHandle n;

	velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	ros::Subscriber sub = n.subscribe("blobs", 1000, blobCallback);   

	ros::Rate loop_rate(10);

	ros::spin();


	return 0;
}



