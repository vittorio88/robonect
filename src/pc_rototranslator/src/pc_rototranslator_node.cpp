/*
 * pc_rototranslator.cpp
 *
 *  Created on: Jun 1, 2012
 *      Author: vittorio
 *      Notes: Implementation of getTf function is flawed. It is useful for cleanliness (currently not used)
 *             Consider converting the whole thing to multiple services (ie this turns into pc_reformatter, and pc_rototranslator and pc_sensor_zero turned into services)
 *			   Consider advantages of not generating rototranslatedpc and publishing old rototranslatedpc vs publishing not publishing anything.
 */


#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>



class pc_rototranslator
{

public:


	pc_rototranslator() : tfTransformListener(),  target_frame("/base_link")
	{

		ROS_INFO("Entering Constructor!!!");


		// Subscribes pcSubscriber to topic, and calls cloud_cb
		pcSubscriber.subscribe(node, "/camera/depth/points", 30);
		pcSubscriber.registerCallback(boost::bind(&pc_rototranslator::cloud_cb, this, _1));// Binds address of cloud_cb to a boost address. Must later be called by function and dereferenced within.

		// Transform Filter for tfTransformListener
		tfMessageFilter = new tf::MessageFilter<geometry_msgs::TransformStamped>(tfSubscriber, tfTransformListener, target_frame, 10);

		// Advertise new cloud
		rototranslatedpcPublisher = node.advertise<sensor_msgs::PointCloud2>("rototranslatedpc", 10);


	} ;

	// DECLARATIONS
	ros::NodeHandle node; // Node name is "node"
	ros::Publisher rototranslatedpcPublisher; // Publisher for rototranslatedpc
	sensor_msgs::PointCloud2 rototranslatedpc; // new pc Object
	message_filters::Subscriber<sensor_msgs::PointCloud2> pcSubscriber; // sensorPcSubscriber   RENAME TO pcSensorSubscriber
	tf::TransformListener tfTransformListener; // tfTransformListener for storing filtered transforms
	std::string target_frame; // frame for tfMessageFilter
	tf::MessageFilter<geometry_msgs::TransformStamped> * tfMessageFilter; // Filter for type geometry_msgs::TransformStamped
	message_filters::Subscriber<geometry_msgs::TransformStamped> tfSubscriber; // tfSubscriber for type geometry_msgs::TransformStamped

	//  Higher level sensorCloud Callback Function
	void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& sensorCloudPtr){

				ROS_INFO("Performing Transform!!!");
				try{
					tfTransformListener.waitForTransform("/camera_link", "/base_link", sensorCloudPtr->header.stamp, ros::Duration(3.0));
					pcl_ros::transformPointCloud ("/base_link", *sensorCloudPtr, rototranslatedpc, tfTransformListener);
				}
				catch (tf::TransformException& ex){
					ROS_ERROR("%s",ex.what());
				}
				ROS_INFO("rototranslatedpc.header.frame_id is"); std::cout<<rototranslatedpc.header.frame_id<<std::endl;

				rototranslatedpcPublish();
	}


	// Publish new rototranslatedpc
	void rototranslatedpcPublish(void){

		//rototranslatedpc=*sensorCloudPtr; // uncomment for input/output of pc

		// Check for subscribers
		if (rototranslatedpcPublisher.getNumSubscribers() > 0)
		{
			rototranslatedpcPublisher.publish(rototranslatedpc);
		}
	}




};

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "pc_rototranslator"); // Init ROS
	pc_rototranslator pc_rototranslator_OBJECT; // Instance Object
	ros::spin(); // Run until interrupted
}
