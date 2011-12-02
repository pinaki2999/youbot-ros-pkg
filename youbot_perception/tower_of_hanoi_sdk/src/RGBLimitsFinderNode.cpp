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

//ROS specific Headers
#include "ros/ros.h"
#include "tower_of_hanoi_sdk/tower_of_hanoi_sdkConfig.h"
#include "dynamic_reconfigure/server.h"

//PCL specific Headers
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"

//BRICS_3D specific Headers
#include "examples/ColorBasedRoiExtractor_RGB.h"

//Sytem-wide Standard Headers
#include <fstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>


/**
 * usage: rosrun tower_of_hanoi_sdk hsvLimitsFinder
 *
 * start the dynamic reconfigure using: rosrun dynamic_configure reconfigure_gui
 * to play with the HSV limits.
 * use Ctrl+c on the ros-node to stop the extraction once the appropriate HSV limits are set
 * save the last found configuration if you want to
 *
 * Topic published:
 *  extracted_region_1
 *  type: Pointcloud2 message
 *  frame_id: /openni_rgb_optical_frame
 */

using namespace std;

//Global Variables
BRICS_3D::ColorBasedRoiExtractor roiExtractor;
int redLimit, greenLimit, blueLimit;
float distanceThresholdMinimum, distanceThresholdMaximum;
ofstream configFileStream;

//ToDo Usage details

/**
 * Call back for Kinect Data
 * @param cloud	received point-cloud from Kinect
 */
void kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud){
	roiExtractor.initializeLimits(redLimit, greenLimit, blueLimit, distanceThresholdMinimum, distanceThresholdMaximum);
	roiExtractor.kinectCloudCallback(cloud);
}


/**
 *Callback for dynamic reconfigure
 */
void callback(tower_of_hanoi_sdk::tower_of_hanoi_sdkConfig &config, uint32_t level)
{
	redLimit = config.limit_red;
	greenLimit = config.limit_green;
	blueLimit = config.limit_blue;
	distanceThresholdMinimum = config.distance_threshold_minimum;
	distanceThresholdMaximum = config.distance_threshold_maximum;
}


/**
 * Saves the last known configuration for HSV limits
 * @param configFileStream	species location to write the configuration
 * @return	true if saving successful
 */
int saveConfig(ofstream &configFileStream){
	//Todo recheck if new config should be saved or not
	configFileStream
	<< "limit_red=" 				<< redLimit    			<<endl
	<< "limit_green=" 				<< greenLimit 			<<endl
	<< "limit_blue="      		<< blueLimit 			<<endl
	<< "distance_threshold_minimum="      		<< distanceThresholdMinimum 			<<endl
	<< "distance_threshold_maximum="      		<< distanceThresholdMaximum 			<<endl;
	cout<< endl<<"New RGB Configuration Saved......"<<endl;

	return 1;
}


int main(int argc, char* argv[]){

	//initialize the ros node
	ros::init(argc, argv, "HSV_LimitsFinder");
	ros::NodeHandle nh;

	//set up the dynamic configure
	dynamic_reconfigure::Server<tower_of_hanoi_sdk::tower_of_hanoi_sdkConfig> srv;
	dynamic_reconfigure::Server<tower_of_hanoi_sdk::tower_of_hanoi_sdkConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	srv.setCallback(f);

	//Define the publishers for each extracted region
	ros::Publisher extractedRoiPublisher;


	//define the HSV limit variables;
	redLimit=0; greenLimit=0; blueLimit=0; distanceThresholdMinimum=0.0; distanceThresholdMaximum=0.0;

	std::ofstream processingLogs, frameDelayLogs;
	processingLogs.open("demo_processing.log");
	frameDelayLogs.open("demo_frame_delay.log");

	roiExtractor.setProcessingLogs(&processingLogs);
	roiExtractor.setFrameDelayLogs(&frameDelayLogs);

	//initalize the roiExtractor
	roiExtractor.initializeLimits(redLimit, greenLimit, blueLimit, distanceThresholdMinimum, distanceThresholdMaximum);

	//initialize the publisher for extracted region
	extractedRoiPublisher = nh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >  ("extracted_region_1", 1);
	roiExtractor.setExtractedRegionPublisher(&extractedRoiPublisher);

	//subscribe to kinect point cloud messages
	ros::Subscriber  kinectCloudSubscriber = nh.subscribe("/camera/rgb/points", 1,&kinectCloudCallback);

	ROS_INFO("Now extracting ROIs ;)");

	//start ros-message callbacks
	ros::spin();

	//Store the configuration
	string userInput;
	cout << "Do you want to save the last configuration of RGB Limits ??  "<< endl;
	cin  >> userInput;
	if(!userInput.compare("y") || !userInput.compare("Y") || !userInput.compare("yes")){
		cout << "Please enter the target file (including path to file) "<< endl;
		cin  >> userInput;
		ofstream configFileStream;
		configFileStream.open(userInput.c_str(), ios::out);
		if(configFileStream.is_open()) saveConfig(configFileStream);
	}

	return 0;
}
