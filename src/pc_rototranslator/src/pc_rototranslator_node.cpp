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

// For segmentation of ground plane, estimation of transform, and transform to align 0 plane with ground for kinect axis of rotation.
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

// Find surface normal
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
class pc_rototranslator
{

public:


	pc_rototranslator() : tfTransformListener(),  target_frame("/base_link")
	{
		ROS_INFO("Entering Constructor!!!");

		// Subscribes pcSubscriber to topic, and calls cloud_cb
		pcSubscriber.subscribe(node, "/camera/depth/points", 3);
		pcSubscriber.registerCallback(boost::bind(&pc_rototranslator::rosCloudCallback, this, _1));// Binds address of cloud_cb to a boost address. Must later be called by function and dereferenced within.

		// Transform Filter for tfTransformListener
		tfMessageFilter = new tf::MessageFilter<geometry_msgs::TransformStamped>(tfSubscriber, tfTransformListener, target_frame, 3);

		// Advertise new cloud
		rototranslatedpcPublisher = node.advertise<sensor_msgs::PointCloud2>("rototranslatedpc", 3);
		viewer_= createVisualizer();
		visualizer_thread_.reset(new boost::thread(boost::bind(&pc_rototranslator::spinVisualizer,this)));


	} ;

	// DECLARATIONS
	ros::NodeHandle node;
	ros::Publisher rototranslatedpcPublisher;
	message_filters::Subscriber<sensor_msgs::PointCloud2> pcSubscriber; // sensorPcSubscriber   RENAME TO pcSensorSubscriber
	tf::TransformListener tfTransformListener; // tfTransformListener for storing filtered transforms
	std::string target_frame; // frame for tfMessageFilter
	tf::MessageFilter<geometry_msgs::TransformStamped> * tfMessageFilter; // Filter for type geometry_msgs::TransformStamped
	message_filters::Subscriber<geometry_msgs::TransformStamped> tfSubscriber; // tfSubscriber for type geometry_msgs::TransformStamped
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
	boost::shared_ptr<boost::thread> visualizer_thread_;

	//  Higher level sensorCloud Callback Function
	int rosCloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& rosCloudPtr){


		pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloudPtr (new pcl::PointCloud<pcl::PointXYZ>  );
		pcl::fromROSMsg( *rosCloudPtr , *tempCloudPtr );
		updateVisualizer(tempCloudPtr);


		//		filterCloud(outputCloudPtr);
		//		transformCloud(outputCloudPtr);
		//		segmentGroundPlane(outputCloudPtr);
		//		findCloudNormal(outputCloudPtr);
		//		rosCloudPublish(outputCloudPtr);
		return 0;

	}


	int filterCloud(const boost::shared_ptr<sensor_msgs::PointCloud2>& inputCloudPtr){
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
		pcl::toROSMsg(*outputPclCloudPtr,*inputCloudPtr); // Replace contents of inputCloudPtr address location with filtered cloud
		return 0;
	}

	// Publish new rototranslatedpc
	int transformCloud(const boost::shared_ptr<sensor_msgs::PointCloud2>& inputCloudPtr){
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
		return 0;
	}

	int segmentGroundPlane(const boost::shared_ptr<sensor_msgs::PointCloud2>& inputCloudPtr){

		// Bring cloud in from ROS
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>  );
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>  );
		//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>  );
		pcl::fromROSMsg( *inputCloudPtr , *cloud_filtered );

		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

		//pcl::PointIndices::Ptr inliers (new pcl::PointIndices ()); // ORIG DONT TOUCH
		const boost::shared_ptr<pcl::PointIndices> inliers (new pcl::PointIndices ());


		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (1000);
		seg.setDistanceThreshold (0.01);

		// Create the filtering object
		pcl::ExtractIndices<pcl::PointXYZ> extract;

		int i = 0, nr_points = (int) cloud_filtered->points.size ();
		// While 30% of the original cloud is still there
		while (cloud_filtered->points.size () > 0.3 * nr_points)
		{
			// Segment the largest planar component from the remaining cloud
			seg.setInputCloud (cloud_filtered);
			seg.segment (*inliers, *coefficients);
			if (inliers->indices.size () == 0)
			{
				std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}

			// Extract the inliers
			extract.setInputCloud (cloud_filtered);
			extract.setIndices (inliers); // IGNORE ECLIPSE ERROR HERE. COMPILER WORKS.
			extract.setNegative (false);
			extract.filter (*cloud_p);
			std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

			//		    std::stringstream ss;
			//		    ss << "table_scene_lms400_plane_" << i << ".pcd";
			//		    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
			//
			//		    // Create the filtering object
			//		    extract.setNegative (true);
			//		    extract.filter (*cloud_f);
			//		    cloud_filtered.swap (cloud_f);
			//		    i++;
		}

		pcl::toROSMsg(*cloud_p,*inputCloudPtr); // Replace contents of inputCloudPtr address location with filtered cloud
		return 0;
	}

	int findCloudNormal(const boost::shared_ptr<sensor_msgs::PointCloud2>& inputCloudPtr){

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>  );
		pcl::fromROSMsg( *inputCloudPtr , *cloud );

		// estimate normals
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

		pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
		ne.setMaxDepthChangeFactor(0.02f);
		ne.setNormalSmoothingSize(10.0f);
		ne.setInputCloud(cloud);
		ne.compute(*normals);

		//		pclCloud_=*cloud ;
		if(!viewer_->updatePointCloud(cloud, "sample cloud"))// IGNORE ECLIPSE ERROR HERE. COMPILER WORKS.
			viewer_->addPointCloud(cloud,  "sample cloud");// IGNORE ECLIPSE ERROR HERE. COMPILER WORKS.


		//pcl::toROSMsg(*cloud_p,*inputCloudPtr); // Replace contents of inputCloudPtr address location with filtered cloud
		return 0;
	}

	void spinVisualizer(){
		ros::Rate r(5);
		while (!viewer_->wasStopped()){
			viewer_->spinOnce();
			r.sleep();
		}
		std::cout<<"Stopped the Viewer"<<std::endl;
	}
	int updateVisualizer(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr){

		pcl::PointCloud<pcl::PointXYZ>::ConstPtr constCloudPtr (cloudPtr );
		if(!viewer_->updatePointCloud(constCloudPtr, "sample cloud"))// IGNORE ECLIPSE ERROR HERE. COMPILER WORKS.
			viewer_->addPointCloud(constCloudPtr,  "sample cloud");// IGNORE ECLIPSE ERROR HERE. COMPILER WORKS.

		return 0;
	}
	boost::shared_ptr<pcl::visualization::PCLVisualizer>
	createVisualizer()
	{

		pcl::PointCloud<pcl::PointXYZ>::Ptr initCloud (new pcl::PointCloud<pcl::PointXYZ> );
		boost::shared_ptr<pcl::visualization::PCLVisualizer> v
			(new pcl::visualization::PCLVisualizer("3D Visualizer"));
		v->setBackgroundColor(0, 0, 0);
		v->addPointCloud<pcl::PointXYZ> (initCloud, "sample cloud");
		v->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
		v->addCoordinateSystem(1.0);
		v->initCameraParameters();
		//v->registerKeyboardCallback(keyboardEventOccurred);

		return(v);
	}


	// Publish new rototranslatedpc
	int rosCloudPublish(const boost::shared_ptr<sensor_msgs::PointCloud2>& inputCloudPtr){

		// Check for subscribers
		if (rototranslatedpcPublisher.getNumSubscribers() > 0)
		{
			rototranslatedpcPublisher.publish(inputCloudPtr);
		}
		return 0;
	}




};

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "pc_rototranslator"); // Init ROS
	pc_rototranslator pc_rototranslator_OBJECT; // Instance Object
	ros::spin(); // Run until interrupted
}
