#include <ros/ros.h>
#include <ros/publisher.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

//#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>


class tf_broadcaster_kinect
{

public:


	tf_broadcaster_kinect()
	{

		ROS_INFO("Entering Constructor!!!");

		// Subscribes to /cur_tilt_angle and registers Callback function
		curTiltAngleSubscriber.subscribe(node, "/cur_tilt_angle", 3);
		curTiltAngleSubscriber.registerCallback(boost::bind(&tf_broadcaster_kinect::curTiltAngleCallback, this, _1));

//        // Subscribes to /cur_tilt_status and registers Callback function
//        curTiltStatusSubscriber.subscribe(node, "/cur_tilt_status", 3);
//        curTiltStatusSubscriber.registerCallback(boost::bind(&tf_broadcaster_kinect::curTiltStatusCallback, this, _1));

		// Subscribes sensorCloud and registers Callback function
		sensorCloudSubscriber.subscribe(node, "/camera/depth/points", 3);
		sensorCloudSubscriber.registerCallback(boost::bind(&tf_broadcaster_kinect::sensorCloudCallback, this, _1));
	}

	// DECLARATIONS
	ros::NodeHandle node;
	tf::TransformBroadcaster tfTransformBroadcaster;
	message_filters::Subscriber<std_msgs::Float64> curTiltAngleSubscriber;
	//message_filters::Subscriber<std_msgs::UInt8> curTiltStatusSubscriber;
	message_filters::Subscriber<sensor_msgs::PointCloud2> sensorCloudSubscriber;
	float curTiltAngle;
//	int curTiltStatus;


void curTiltAngleCallback(const boost::shared_ptr<const std_msgs::Float64>& curTiltAngleMsg){
	curTiltAngle = curTiltAngleMsg->data;
//	ROS_INFO("curTiltAngle is %f",curTiltAngle); // NOISY!
}


//void curTiltStatusCallback(const boost::shared_ptr<const std_msgs::UInt8>& curTiltStatusMsg){
//			curTiltStatus = curTiltStatusMsg->data; // not used
//}


void sensorCloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& sensor_cloud)
	{

	// Check if kinect is moving, and apply a filter discarding updates of less than 1.1 difference.
//	if( std::abs(std::abs(curTiltAngle)-std::abs(prevTiltAngle)) > 1.1 ){

	// curTiltAngle is incorrect if it is less than -50
	if (curTiltAngle > -50){


	// Broadcast transform.
	tfTransformBroadcaster.sendTransform(
			tf::StampedTransform(
					tf::Transform(
							tf::Quaternion( 0, std::sin( (curTiltAngle*-1)*(M_PI/360.0) ), 0, std::cos( (curTiltAngle*-1)*(M_PI/360.0) ) ) , // WORKING!
							//tf::Quaternion( 0, std::cos( int((curTiltAngle)*(M_PI/360.0)) ), 0, std::sin( int((curTiltAngle)*(M_PI/360.0)) ) ) ,
							tf::Vector3(0.0, 0.0, 0.45)),
							sensor_cloud->header.stamp,
							"/base_link",
							"/camera_link"));

	ROS_INFO("New Transform Published!!! Cloud sequence ID is:%d The /cur_tilt_angle is:%f.",sensor_cloud->header.seq, curTiltAngle );

	}}
//	}

};
int main(int argc, char** argv){
	ros::init(argc, argv, "tf_broadcaster_kinect");
	tf_broadcaster_kinect tf_broadcaster_kinect_OBJECT; // Instance Object
	ros::spin();
}
