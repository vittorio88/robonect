/*
 * kinect_teleop_fixedangle.cpp
 *
 *  Created on: Jun 11, 2012
 *      Author: vittorio
 *      Description: This programs publishes to tilt angle to move kinect.
 *
 */


#include <ros/ros.h>
#include <std_msgs/Float64.h>

ros::Publisher kinectAnglePublisher;
std_msgs::Float64 kinectAngle;

int main(int argc, char** argv){
	ros::init(argc, argv, "kinect_teleop_fixedangle");
	ros::NodeHandle node;
	ros::Rate r(3);


	kinectAnglePublisher = node.advertise<std_msgs::Float64>("tilt_angle", 3);
	kinectAngle.data = -10.0; // set angle to -10 deg

	while(node.ok()){
		kinectAnglePublisher.publish(kinectAngle);
		r.sleep();
	}
}
