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

#include "examples/ColorBasedEuclideanClusterExtractor.h"
#include "Color.h"

//ROS specific Headers
#include <ros/ros.h>

//PCL specific Headers
#include <pcl_ros/point_cloud.h>
#include "pcl/point_types.h"

//Sytem-wide Standard Headers
#include <fstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>
#include <sstream>
#include <vector>
//ToDo
/**
 * In current version the the known colors are intialized in
 * ColorBasedEuclideanClusterExtractor::initializeLimits(...)
 *
 * We need to read in a configuration to load the list of
 * known colors and number of known colors
 *  */

int main(int argc, char* argv[]){


		ros::init(argc, argv, "ObjectClusterExtractor");
		ros::NodeHandle nh;

		int maxNoOfObjects;

		int noOfKnownColors = 3;	// default value for red-green-blue

		BRICS_3D::ColorBasedEuclideanClusterExtractor colorBasedEuclideanClusterExtractor;

		if(argc == 2){
			maxNoOfObjects = atoi(argv[2]);
		} else {
			ROS_INFO("Using default values");
			maxNoOfObjects = 1;
		}

	ROS_INFO("Looking for at most [%d] object-cluster(s) for every distinct known-color "
			"in the input image", maxNoOfObjects);


	//Define the publishers for each extracted region
	ros::Publisher extractedClusterPublisher[noOfKnownColors*maxNoOfObjects];
	BRICS_3D::Color red,green,yellow;
	std::vector<BRICS_3D::Color> knownColors;
	red.setColor(255,0,0,"RED");
	green.setColor(00,200,0,"GREEN");
	yellow.setColor(255,255,0,"YELLOW");
	knownColors.push_back(red);
	knownColors.push_back(green);
	knownColors.push_back(yellow);

	colorBasedEuclideanClusterExtractor.setExtractedRegionPublisher(extractedClusterPublisher);
	colorBasedEuclideanClusterExtractor.initializeLimits(3,knownColors,50,0.5,100,2000,3);
	ros::Subscriber kinectSubscriber= nh.subscribe("/camera/rgb/points", 1,
	    			&BRICS_3D::ColorBasedEuclideanClusterExtractor::kinectCloudCallback, &colorBasedEuclideanClusterExtractor);

	ros::spin();
	return 0;
}


