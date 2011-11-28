/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
*
* Author: Pinaki Sunil Banerjee
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and Modified BSD license. The dual-license implies that
* users of this code may choose which terms they prefer.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for
* more details.
*
******************************************************************************/
#include "EuclideanClusterExtractor.h"

namespace BRICS_3D{

EuclideanClusterExtractor::EuclideanClusterExtractor() {
	centroid3DEstimator = new BRICS_3D::Centroid3D();
	maxNoOfObjects = 3;
	this->noOfFramesProcessed=0;
}

ros::Publisher *EuclideanClusterExtractor::getExtractedClusterPublisher() const
{
	return extractedClusterPublisher;
}

int EuclideanClusterExtractor::getMaxNoOfObjects() const
{
	return maxNoOfObjects;
}

void EuclideanClusterExtractor::setMaxNoOfObjects(int maxNoOfObjects)
{
	this->maxNoOfObjects = maxNoOfObjects;
	this->noOfFramesProcessed=0;
}

void EuclideanClusterExtractor::setExtractedClusterPublisher(ros::Publisher *extractedClusterPublisher){
	this->extractedClusterPublisher = extractedClusterPublisher;
}

EuclideanClusterExtractor::~EuclideanClusterExtractor() {
	delete centroid3DEstimator;
}


void EuclideanClusterExtractor::kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud){

	//ToDo assert if every thing is initialized or not

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz_rgb_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

    //Transform sensor_msgs::PointCloud2 msg to pcl::PointCloud
    pcl::fromROSMsg (cloud, *cloud_xyz_rgb_ptr);

    if(cloud_xyz_rgb_ptr->size() > this->euclideanClusterExtractor.getMinClusterSize()){


    	if(this->noOfFramesProcessed!=1 && this->noOfFramesProcessed<100 ){
    			*frameDelayLogs << this->noOfFramesProcessed << "\t"<<  cloud_xyz_rgb_ptr->size() << "\t" <<
    							frameDelayTimer.elapsed()<< "\n";

    			frameDelayTimer.restart();

    	} else {
    		frameDelayTimer.restart();
    	}

        BRICS_3D::PointCloud3D *in_cloud = new BRICS_3D::PointCloud3D();
        std::vector<BRICS_3D::PointCloud3D*> extracted_clusters;
    // cast PCL to BRICS_3D type
    pclTypecaster.convertToBRICS3DDataType(cloud_xyz_rgb_ptr, in_cloud);

    //-------------------------------------------------------------------------------------------
    //extract the clusters
    processingTimer.restart();
    euclideanClusterExtractor.extractClusters(in_cloud, &extracted_clusters);

    ROS_INFO("No of clusters found: %d", extracted_clusters.size());
	if(noOfFramesProcessed<100)
	*processingLogs << this->noOfFramesProcessed << "\t" << cloud_xyz_rgb_ptr->size() << "\t"<<
						processingTimer.elapsed() <<
						"\t" << extracted_clusters.size() << "\n";
	//-------------------------------------------------------------------------------------------


    //Publish the extracted clusters
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>());
	int regions;
	if(extracted_clusters.size() < abs(maxNoOfObjects)) {
		regions = extracted_clusters.size();
	} else {
		regions = maxNoOfObjects;
	}
    for (int i = 0; i < regions; i++){

    	if(extracted_clusters[i]->getSize() > 0){
    	Eigen::Vector3d centroid3d = centroid3DEstimator->computeCentroid(extracted_clusters[i]);

    	pclTypecaster.convertToPCLDataType(tempCloud, extracted_clusters[i]);
    	tempCloud->header.frame_id = "/openni_rgb_optical_frame";
    	extractedClusterPublisher[i].publish(*tempCloud);

    	ROS_INFO("Cluster Position 3D: [%f, %f, %f]", centroid3d[0], centroid3d[1], centroid3d[2]);

         static tf::TransformBroadcaster br;
         tf::Transform transform;
         transform.setOrigin( tf::Vector3(centroid3d[0], centroid3d[1], centroid3d[2]) );
         //Todo stop using Quaternion
         transform.setRotation( tf::Quaternion(0, 0, 0) );
         br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/openni_rgb_optical_frame",
        		 extractedClusterPublisher[i].getTopic()));
    	}
    }

    tempCloud.reset();
    delete(in_cloud);
    extracted_clusters.clear();
    }

    //		ROS_INFO("no of frames processed %d",noOfFramesProcessed);

    		this->noOfFramesProcessed++;

    		if(noOfFramesProcessed>100){
    			ROS_WARN("Logging Completed");
 //   			exit(0);
    		}


}

void EuclideanClusterExtractor::initializeExtractor(int maxNoObjects,
		ros::Publisher *extractedClusterPublisher, int minClusterSize, int maxClusterSize,
																		float clusterTolerance){

	setMinClusterSize(minClusterSize);
	setMaxClusterSize(maxClusterSize);
	setClusterTolerance(clusterTolerance);
	setMaxNoOfObjects(maxNoObjects);
	setExtractedClusterPublisher(extractedClusterPublisher);

}

}
