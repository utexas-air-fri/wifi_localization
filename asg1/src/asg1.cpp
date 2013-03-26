#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "cmvision/Blobs.h"

ros::Publisher velocity_pub;

// This method is called whenever a blob message is received
void blobCallback(const cmvision::Blobs::ConstPtr& msg){

  // This is the output velocity that we will publish
  geometry_msgs::Twist output;

  // This is an example print message
  std::cout << "got blob message, count: " << msg->blob_count << std::endl;

  // first, we can check if any blobs were found
  // blobs are regions of a single color found in the camera image
  if (msg->blob_count > 0){

    // we may want to access / look at multiple blobs in the array
    // this array is all the blobs found in the latest camera image
    // you may have multiple blobs because there were multiple balls
    // but also because the ball may have been broken into multiple blobs
    for (int i = 0; i < msg->blob_count; i++){

      // another example print with some blob info
      std::cout << "Detected blob " << i << " with area " << msg->blobs[i].area << std::endl;

      // some things to look at
      msg->blobs[i].area;      // blob area

      msg->blobs[i].x;         // blob center x
      msg->blobs[i].y;         // blob center y

      msg->blobs[i].left;      // blob left x
      msg->blobs[i].right;     // blob right x
      msg->blobs[i].top;       // blob top x
      msg->blobs[i].bottom;    // blob bottom x
      
      // it's possible you will see multiple balls, or multiple detected
      // blobs within 1 ball... you may want to only use info from the biggest one

    }

    // TODO: decide what velocities to publish based on blob info

    // you probably want to set these to zero when you do not see any blobs
    // you may want to work on turning first, and then fwd/backward

    output.linear.x = 0; // TODO: fill in this with some number for fwd velocity (meters/sec)
    output.angular.z = 0; // TODO: fill this in with some angular velocity (radians/sec)

    velocity_pub.publish(output); // publish this velocity message that we filled in
  }

}


int main(int argc, char **argv){

	ros::init(argc, argv, "follower");
	ros::NodeHandle n;

	// advertise that we will publish cmd_vel messages
	velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	// subscribe to blob messages and call blobCallback when they are received
	ros::Subscriber sub = n.subscribe("blobs", 1000, blobCallback);   

	ros::Rate loop_rate(10);

	ros::spin();


	return 0;
}


