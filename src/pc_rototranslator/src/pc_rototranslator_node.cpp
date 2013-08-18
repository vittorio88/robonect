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
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>



class pc_rototranslator
{

public:


	pc_rototranslator() : tfTransformListener(),  target_frame("/base_link")
	{

		ROS_INFO("Entering Constructor!!!");


		// Subscribes pcSubscriber to topic, and calls cloud_cb
		pcSubscriber.subscribe(node, "/camera/depth/points", 3);
		pcSubscriber.registerCallback(boost::bind(&pc_rototranslator::cloud_cb, this, _1));// Binds address of cloud_cb to a boost address. Must later be called by function and dereferenced within.

		// Transform Filter for tfTransformListener
		tfMessageFilter = new tf::MessageFilter<geometry_msgs::TransformStamped>(tfSubscriber, tfTransformListener, target_frame, 3);

		// Advertise new cloud
		rototranslatedpcPublisher = node.advertise<sensor_msgs::PointCloud2>("rototranslatedpc", 3);


	} ;

	// DECLARATIONS
	ros::NodeHandle node; // Node name is "node"
	ros::Publisher rototranslatedpcPublisher; // Publisher for rototranslatedpc
	message_filters::Subscriber<sensor_msgs::PointCloud2> pcSubscriber; // sensorPcSubscriber   RENAME TO pcSensorSubscriber
	tf::TransformListener tfTransformListener; // tfTransformListener for storing filtered transforms
	std::string target_frame; // frame for tfMessageFilter
	tf::MessageFilter<geometry_msgs::TransformStamped> * tfMessageFilter; // Filter for type geometry_msgs::TransformStamped
	message_filters::Subscriber<geometry_msgs::TransformStamped> tfSubscriber; // tfSubscriber for type geometry_msgs::TransformStamped

	//  Higher level sensorCloud Callback Function
	void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& sensorCloudPtr){


		boost::shared_ptr<sensor_msgs::PointCloud2> outputCloudPtr (new sensor_msgs::PointCloud2 );
		*outputCloudPtr=*sensorCloudPtr;

		filterCloud(outputCloudPtr);
		transformCloud(outputCloudPtr);
		publishCloud(outputCloudPtr);

	}


	void filterCloud(const boost::shared_ptr<sensor_msgs::PointCloud2>& inputCloudPtr){
		pcl::PointCloud<pcl::PointXYZ>::Ptr inputPclCloudPtr (new pcl::PointCloud<pcl::PointXYZ>  );
		pcl::PointCloud<pcl::PointXYZ>::Ptr outputPclCloudPtr (new pcl::PointCloud<pcl::PointXYZ>  );
		pcl::PointCloud<pcl::PointXYZ>::Ptr voxelPclCloudPtr (new pcl::PointCloud<pcl::PointXYZ>  );
		pcl::PointCloud<pcl::PointXYZ>::Ptr sorPclCloudPtr (new pcl::PointCloud<pcl::PointXYZ>  );
		pcl::fromROSMsg( *inputCloudPtr , *inputPclCloudPtr );

		// Perform voxel filter
		pcl::VoxelGrid<pcl::PointXYZ> vox;
		vox.setInputCloud (inputPclCloudPtr);
		vox.setLeafSize (0.01f, 0.01f, 0.01f);
		vox.filter (*voxelPclCloudPtr);

		// Create the filtering object
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud (voxelPclCloudPtr);
		sor.setMeanK (50);
		sor.setStddevMulThresh (1.0);
		sor.filter (*sorPclCloudPtr);


		*outputPclCloudPtr=*sorPclCloudPtr;
		ROS_INFO("Filter: incloud height=%d", inputCloudPtr->height );
		ROS_INFO("Filter: incloud width=%d", inputCloudPtr->width );
		ROS_INFO("Filter: outcloud height=%d", outputPclCloudPtr->height );
		ROS_INFO("Filter: outcloud width=%d", outputPclCloudPtr->width );
		ROS_INFO("Filter: voxelPclCloudPtr height=%d", voxelPclCloudPtr->height );
		ROS_INFO("Filter: voxelPclCloudPtr width=%d", voxelPclCloudPtr->width );
		ROS_INFO("Filter: sorPclCloudPtr height=%d", sorPclCloudPtr->height );
		ROS_INFO("Filter: sorPclCloudPtr width=%d", sorPclCloudPtr->width );
		pcl::toROSMsg(*outputPclCloudPtr,*inputCloudPtr); // Replace contents of inputCloudPtr address location with filtered cloud

	}

	// Publish new rototranslatedpc
	void transformCloud(const boost::shared_ptr<sensor_msgs::PointCloud2>& inputCloudPtr){
		sensor_msgs::PointCloud2 tempCloud;
		try{
			tfTransformListener.waitForTransform("/camera_link", "/base_link", inputCloudPtr->header.stamp, ros::Duration(3.0));
			pcl_ros::transformPointCloud ("/base_link", *inputCloudPtr, tempCloud, tfTransformListener);
		}
		catch (tf::TransformException& ex){
			ROS_ERROR("%s",ex.what());
		}
		ROS_INFO("Transform: The cloud.header.frame_id is: %s ",tempCloud.header.frame_id.c_str());
		*inputCloudPtr=tempCloud;
	}



	// Publish new rototranslatedpc
	void publishCloud(const boost::shared_ptr<sensor_msgs::PointCloud2>& inputCloudPtr){

		// Check for subscribers
		if (rototranslatedpcPublisher.getNumSubscribers() > 0)
		{
			rototranslatedpcPublisher.publish(inputCloudPtr);
		}
	}




};

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "pc_rototranslator"); // Init ROS
	pc_rototranslator pc_rototranslator_OBJECT; // Instance Object
	ros::spin(); // Run until interrupted
}
