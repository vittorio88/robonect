/*
 * kinect_teleop_joy.cpp
 *
 *  Created on: Jun 11, 2012
 *      Author: vittorio
 *      Description: This programs subscribes to joy topic, which corresponds to xbox left analog stick, and publishes to tilt_angle topic.
 *
 */


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>

class kinect_teleop_joy
{
public:
	kinect_teleop_joy();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle node;
	ros::Publisher kinectAnglePublisher;
	ros::Subscriber joySubscriber;
	std_msgs::Float64 kinectAngle;
};


kinect_teleop_joy::kinect_teleop_joy()
{
	joySubscriber = node.subscribe<sensor_msgs::Joy>("joy", 1, &kinect_teleop_joy::joyCallback, this);
	kinectAnglePublisher = node.advertise<std_msgs::Float64>("tilt_angle", 30);
}

void kinect_teleop_joy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	kinectAngle.data = 30*joy->axes[1] - 4 ; // Gain and offset
	ROS_INFO("Entering joyCallback. kinectAngle.data=%f",kinectAngle.data);
	kinectAnglePublisher.publish(kinectAngle);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "kinect_teleop_joy");
	kinect_teleop_joy kinect_teleop_joy_OBJECT;
	ros::spin();
}
