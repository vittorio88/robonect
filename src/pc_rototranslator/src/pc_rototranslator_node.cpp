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
#include <tf_conversions/tf_eigen.h>
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

// For printing uint64
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

class pc_rototranslator
{

public:


	pc_rototranslator() : tfTransformListener_(),  target_frame_("/base_link")
	{
		ROS_INFO("Entering Constructor!!!");

		// Subscribes pcSubscriber to topic, and calls cloud_cb
		cloudSubscriber_.subscribe(node_, "/camera/depth/points", 3);
		cloudSubscriber_.registerCallback(boost::bind(&pc_rototranslator::rosCloudCallback, this, _1));// Binds address of cloud_cb to a boost address. Must later be called by function and dereferenced within.

		// Transform Filter for tfTransformListener
		tfMessageFilter_ = new tf::MessageFilter<geometry_msgs::TransformStamped>(tfSubscriber_, tfTransformListener_, target_frame_, 3);

		// Advertise new cloud
		rototranslatedpcPublisher_ = node_.advertise<sensor_msgs::PointCloud2>("rototranslatedpc", 3);
		//viewer_= createVisualizer();
		//visualizer_thread_.reset(new boost::thread(boost::bind(&pc_rototranslator::spinVisualizer,this)));


	} ;

	// DECLARATIONS
	ros::NodeHandle node_;
	ros::Publisher rototranslatedpcPublisher_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSubscriber_;
	tf::TransformListener tfTransformListener_;
	std::string target_frame_;
	tf::MessageFilter<geometry_msgs::TransformStamped> * tfMessageFilter_;
	message_filters::Subscriber<geometry_msgs::TransformStamped> tfSubscriber_;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
	boost::shared_ptr<boost::thread> visualizer_thread_;

	//  Higher level sensorCloud Callback Function
	int
	rosCloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& rosCloudPtr){


		pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloudPtr (new pcl::PointCloud<pcl::PointXYZ>  );
		pcl::fromROSMsg( *rosCloudPtr , *tempCloudPtr );
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr constCloudPtr (tempCloudPtr );



		// filter cloud
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr filteredCloud =  filterCloud(constCloudPtr);
		transformCloud(constCloudPtr,rosCloudPtr->header.stamp);
		timespec s_crt, e_crt;


		// TODO: EXPERIMENTS WITH TIME STAMPS. TRY TO PLACE ROS TIME STAMP IN POINTCLOUD HEADER
		//constCloudPtr->header.stamp = gettimeofday(&s_get, NULL);
//		constCloudPtr->header.stamp = clock_gettime(CLOCK_REALTIME,  &s_crt);
//		ROS_INFO("#######" );
//		ROS_INFO("rosCloudPtr->header.stamp = ", rosCloudPtr->header.stamp.sec );
//		ROS_INFO("constCloudPtr->header.stamp = "PRIu64, constCloudPtr->header.stamp );
//		ROS_INFO("ros::Time(constCloudPtr->header.stamp) = "PRIu64, ros::Time(constCloudPtr->header.stamp) );
//		ROS_INFO("#######");




		// find normals
		//pcl::PointCloud<pcl::PointXYZ>::ConstPtr groundPlaneCloud = segmentGroundPlane(filteredCloud);
		//pcl::PointCloud<pcl::Normal>::ConstPtr normals = findCloudNormal(groundPlaneCloud);

		// publish and visualize cloud
		//updateVisualizer(filterCloud, normals);
		rosCloudPublish(filteredCloud);
		return 0;

	}


	pcl::PointCloud<pcl::PointXYZ>::ConstPtr
	filterCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& inputCloudPtr){

		pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloudPtr (new pcl::PointCloud<pcl::PointXYZ>  );
		pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloudPtr (new pcl::PointCloud<pcl::PointXYZ>  );
		pcl::PointCloud<pcl::PointXYZ>::Ptr sorCloudPtr (new pcl::PointCloud<pcl::PointXYZ>  );

		// Perform voxel filter
		pcl::VoxelGrid<pcl::PointXYZ> vox;
		vox.setInputCloud (inputCloudPtr);
		vox.setLeafSize (0.01f, 0.01f, 0.01f);
		vox.filter (*voxelCloudPtr);

		// Create the filtering object
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud (voxelCloudPtr);
		sor.setMeanK (50);
		sor.setStddevMulThresh (1.0);
		sor.filter (*sorCloudPtr);




		*outputCloudPtr=*sorCloudPtr; // select output cloud
		ROS_INFO("Filter: incloud height=%d", inputCloudPtr->height );
		ROS_INFO("Filter: incloud width=%d", inputCloudPtr->width );
		ROS_INFO("Filter: outcloud height=%d", outputCloudPtr->height );
		ROS_INFO("Filter: outcloud width=%d", outputCloudPtr->width );
		return outputCloudPtr;
	}

	// Publish new rototranslatedpc
	pcl::PointCloud<pcl::PointXYZ>::Ptr
//	transformCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& inputCloudPtr){
	transformCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& inputCloudPtr, ros::Time sensorStamp){

		pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloudPtr (new pcl::PointCloud<pcl::PointXYZ>  );
	      tf::StampedTransform tfTransform;
	      Eigen::Affine3d eigenTransform;


		try{
			//tfTransformListener_.waitForTransform("/camera_link", "/base_link", inputCloudPtr->header.stamp, ros::Duration(3.0));// ORIG DOESNT WORK
			tfTransformListener_.waitForTransform("/camera_link", "/base_link", sensorStamp, ros::Duration(3.0));
			tfTransformListener_.lookupTransform("/camera_link", "/base_link", sensorStamp, tfTransform);
			tf::transformTFToEigen(tfTransform, eigenTransform);
			pcl::transformPointCloud(*inputCloudPtr, *outputCloudPtr, eigenTransform);
			//pcl_ros::transformPointCloud ("/base_link", *inputCloudPtr, outputCloudPtr, tfTransformListener_); // CHANGE TO GET TRANSFORM AND APPLY IN PCL
		}
		catch (tf::TransformException& ex){
			ROS_ERROR("%s",ex.what());
		}

		ROS_INFO("Transform: The cloud.header.frame_id is: %s ",outputCloudPtr->header.frame_id.c_str());
		return outputCloudPtr;
	}

	pcl::PointCloud<pcl::PointXYZ>::ConstPtr
	segmentGroundPlane(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& inputCloudPtr){

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>  );
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>  );


		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

		pcl::PointIndices::Ptr inliers (new pcl::PointIndices ()); // ORIG DONT TOUCH
		//const boost::shared_ptr<pcl::PointIndices> inliers (new pcl::PointIndices ());


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

		int i = 0, nr_points = (int) inputCloudPtr->points.size ();
		// While 30% of the original cloud is still there
		while (inputCloudPtr->points.size () > 0.3 * nr_points)
		{
			// Segment the largest planar component from the remaining cloud
			seg.setInputCloud (inputCloudPtr);
			seg.segment (*inliers, *coefficients);
			if (inliers->indices.size () == 0)
			{
				std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}
			pcl::PointIndices::ConstPtr inliersConst (inliers);
			// Extract the inliers
			extract.setInputCloud (inputCloudPtr);
			extract.setIndices (inliersConst);
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


		return cloud_p;
	}

	pcl::PointCloud<pcl::Normal>::ConstPtr
	findCloudNormal(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& inputCloudPtr){

		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
		ne.setMaxDepthChangeFactor(0.02f);
		ne.setNormalSmoothingSize(10.0f);
		ne.setInputCloud(inputCloudPtr);
		ne.compute(*normals);

		pcl::PointCloud<pcl::Normal>::ConstPtr normalsConst (new pcl::PointCloud<pcl::Normal>);
		return normalsConst;
	}

	int
	spinVisualizer(void){
		ros::Rate r(5);
		while (!viewer_->wasStopped()){
			viewer_->spinOnce();
			r.sleep();
		}
		std::cout<<"Stopped the Viewer"<<std::endl;
	return 0;
	}


	int
	updateVisualizer(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
						const pcl::PointCloud<pcl::Normal>::ConstPtr& normals){


		viewer_->updatePointCloud(cloud, "sample cloud");
		viewer_->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 10, 0.05, "normals");


		return 0;
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer>
	createVisualizer( void )
	{

		pcl::PointCloud<pcl::PointXYZ>::Ptr initCloud (new pcl::PointCloud<pcl::PointXYZ> ); //stays empty
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
	int rosCloudPublish(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& inputCloudPtr){

		const boost::shared_ptr<sensor_msgs::PointCloud2> rosCloudPtr (new sensor_msgs::PointCloud2);
		pcl::toROSMsg(*inputCloudPtr,*rosCloudPtr); // Replace contents of inputCloudPtr address location with filtered cloud

		// Check for subscribers
		if (rototranslatedpcPublisher_.getNumSubscribers() > 0)
		{
			rototranslatedpcPublisher_.publish(rosCloudPtr);
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
