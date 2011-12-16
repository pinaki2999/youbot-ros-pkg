/*
 * ColorBasedEuclideanClusterExtractor.cpp
 *
 *  Created on: Dec 15, 2011
 *      Author: pinaki
 */

#include "ColorBasedEuclideanClusterExtractor.h"

namespace BRICS_3D {

ColorBasedEuclideanClusterExtractor::ColorBasedEuclideanClusterExtractor() {
	// TODO Auto-generated constructor stub

}

ColorBasedEuclideanClusterExtractor::~ColorBasedEuclideanClusterExtractor() {
	// TODO Auto-generated destructor stub
}

void ColorBasedEuclideanClusterExtractor::initializeLimits(int noOfKnownColors,  std::vector<BRICS_3D::Color> knownColors, double toleranceRGBSpace,
		double toleranceEuclideanDistance, 	unsigned int minClusterSize, unsigned int maxClusterSize,unsigned int maxNoOfObjects){
	this->coloredClusterExtractor.setMinClusterSize(minClusterSize);
	this->coloredClusterExtractor.setMaxClusterSize(maxClusterSize);
	this->coloredClusterExtractor.setToleranceEuclideanDistance(toleranceEuclideanDistance);
	this->coloredClusterExtractor.setToleranceRgbSpace(toleranceRGBSpace);
	assert(noOfKnownColors == knownColors.size());
	this->knownColors = knownColors;
	this->noOfKnownColors = noOfKnownColors;
	this->maxNoOfObjects = maxNoOfObjects;
}

void ColorBasedEuclideanClusterExtractor::kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud){
	ROS_INFO("Under Development Process");

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz_rgb_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	//Transform sensor_msgs::PointCloud2 msg to pcl::PointCloud
	pcl::fromROSMsg (cloud, *cloud_xyz_rgb_ptr);

	BRICS_3D::PCLTypecaster pclTypecaster;
	BRICS_3D::ColoredPointCloud3D *in_cloud = new BRICS_3D::ColoredPointCloud3D();
	pclTypecaster.convertToBRICS3DDataType(cloud_xyz_rgb_ptr, in_cloud);


	coloredClusterExtractor.setPointCloud(in_cloud);
	coloredClusterExtractor.segment();
	std::vector<BRICS_3D::ColoredPointCloud3D*> extractedClusters;
	coloredClusterExtractor.getExtractedClusters(extractedClusters);

	ROS_INFO("Number of clusters found: %d", extractedClusters.size());

	//Associate each cluster with a known color
	for (int i =0; i<extractedClusters.size(); i++){

		int avgRed, avgGreen, avgBlue;
		findMeanColor(avgRed, avgGreen, avgBlue,extractedClusters[i]);
		double bestSimilarityRGBSpace = DBL_MAX, currentSimilarityRGBSpace;
		int best_color_index=0;
		for(size_t count=0; count < knownColors.size(); count++){
			currentSimilarityRGBSpace = knownColors[count].findSimilarityRGBSpace(avgRed,avgGreen, avgBlue);
			//			ROS_INFO("Similarity Score: [%f] for [%s] Avg. Values=[%d %d %d]", currentSimilarityRGBSpace,
			//												knownColors[count].getName().c_str(), avgRed, avgGreen, avgBlue);
			if(currentSimilarityRGBSpace < bestSimilarityRGBSpace){
				bestSimilarityRGBSpace = currentSimilarityRGBSpace;
				best_color_index = count;
			}
		}
		if(bestSimilarityRGBSpace < coloredClusterExtractor.getToleranceRgbSpace()){
			ROS_INFO("Found Color: [%s] score [%f]  Avg. Values=[%d %d %d] size: [%d]",
					knownColors[best_color_index].getName().c_str(),bestSimilarityRGBSpace,
					avgRed, avgGreen, avgBlue, extractedClusters[i]->getSize() );
			          std::cout << "PointCloud representing the Cluster: " << extractedClusters[i]->getSize() << " data points." << std::endl;

			//Publish the extracted object

			//             cloud_cluster->header.frame_id = "openni_rgb_optical_frame";
			//             pubClusters[i].publish(*cloud_cluster);
			//             ROS_INFO("Object Cluster Size: %d", cloud_cluster->points.size());
//			Eigen::Vector4f centroid3d;
//			pcl::compute3DCentroid(*cloud_cluster,centroid3d);
//			//                         ROS_INFO("Object_%d located at [%f,%f,%f]",i+1, centroid3d[0], centroid3d[1], centroid3d[2]);
//
//			static tf::TransformBroadcaster br;
//			tf::Transform transform;
//			transform.setOrigin( tf::Vector3(centroid3d[0], centroid3d[1], centroid3d[2]) );
//			transform.setRotation( tf::Quaternion(0, 0, 0) );
//			std::stringstream s;
//			s<<knownColors[best_color_index].getName()<<"_"<<(int)bestSimilarityRGBSpace;
//			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "openni_rgb_optical_frame", s.str()));
		}

	}
}

void ColorBasedEuclideanClusterExtractor::findMeanColor(int &red, int &green, int &blue, BRICS_3D::ColoredPointCloud3D* coloredInput){

	red=0;green=0;blue=0;

	for (size_t i =0; i< coloredInput->getSize(); i++){
		red = coloredInput->getPointCloud()->data()[i].red;
		blue = coloredInput->getPointCloud()->data()[i].blue;
		green = coloredInput->getPointCloud()->data()[i].green;
	}

	red = (int) (red/coloredInput->getSize());
	green = (int) (green/coloredInput->getSize());
	blue = (int) (blue/coloredInput->getSize());
}

} /* namespace BRICS_3D */
