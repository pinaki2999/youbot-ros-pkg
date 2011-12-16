/*
 * ColorBasedEuclideanClusterExtractor.h
 *
 *  Created on: Dec 15, 2011
 *      Author: pinaki
 */

#ifndef COLORBASEDEUCLIDEANCLUSTEREXTRACTOR_H_
#define COLORBASEDEUCLIDEANCLUSTEREXTRACTOR_H_

#include "util/PCLTypecaster.h"
#include "RGBColorBasedEuclideanClustering.h"
#include "core/ColoredPointCloud3D.h"
#include "Color.h"
#include <vector>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <float.h>
#include <tf/transform_broadcaster.h>


namespace BRICS_3D {

class ColorBasedEuclideanClusterExtractor {

    BRICS_3D::RGBColorBasedEuclideanClustering coloredClusterExtractor;

    std::vector<BRICS_3D::Color> knownColors;

    void findMeanColor(int &red, int &green, int &blue, BRICS_3D::ColoredPointCloud3D* coloredInput);

    unsigned int noOfKnownColors;

    unsigned int maxNoOfObjects;

    ros::Publisher *extractedRegionPublisher;

public:

    //ToDo add function to add/remove colors from the set of known-colors

	ColorBasedEuclideanClusterExtractor();
	virtual ~ColorBasedEuclideanClusterExtractor();

	/**
	 * Callback function for Kinect data. Finds the ROI from the input data
	 * @param cloud input cloud from Kinect
	 */
	void kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud);


	/**
	 *
	 * @return the current publisher which will be used to publish the extracted regions of interests
	 */
	ros::Publisher* getExtractedRegionPublisher() const;


	/**
	 * sets the current publisher which will be used to publish the extracted regions of interests
	 * @param extractedRegionPublisher the current publisher which will be used to publish the extracted ROIs
	 */
    void setExtractedRegionPublisher(ros::Publisher *extractedRegionPublisher){
    	this->extractedRegionPublisher = extractedRegionPublisher;
    }


    void initializeLimits(int noOfKnownColors,  std::vector<BRICS_3D::Color> knownColors,
    		double toleranceRGBSpace, double toleranceEuclideanDistance,
    		unsigned int minClusterSize, unsigned int maxClusterSize,
    		unsigned int maxNoOfObjects);

};

} /* namespace BRICS_3D */
#endif /* COLORBASEDEUCLIDEANCLUSTEREXTRACTOR_H_ */
