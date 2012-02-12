/*
 * PlaneEstimator.cpp
 *
 *  Created on: Feb 8, 2012
 *      Author: reon
 */



//ROS specific Headers
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "tower_of_hanoi_sdk/Coordination.h"
#include "tower_of_hanoi_sdk/Configuration.h"
#include "ros/publisher.h"

//PCL specific Headers
#include <pcl_ros/point_cloud.h>
#include "pcl/point_types.h"
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//BRICS_3D specific headers
#include "examples/PoseEstimation6D.h"

//Sytem-wide Standard Headers
#include <fstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>
#include <sstream>

void kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud){

	ROS_INFO("Estimating Plane");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());

		//Transform sensor_msgs::PointCloud2 msg to pcl::PointCloud
	pcl::fromROSMsg (cloud, *cloud_xyz);



   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
//  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_xyz);
  seg.segment (*inliers, *coefficients);

//  if (inliers->indices.size () == 0)
//  {
//    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
//    return (-1);
//  }
//
  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " "
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;


}


int main(int argc, char* argv[]){

	ros::init(argc, argv, "PlaneEstimator");
	ros::NodeHandle nh;

	//subscribe to kinect point cloud messages
	ros::Subscriber  kinectCloudSubscriber;
		kinectCloudSubscriber= nh.subscribe("/camera/rgb/points", 1,&kinectCloudCallback);

	ros::spin();

	return 0;
}
