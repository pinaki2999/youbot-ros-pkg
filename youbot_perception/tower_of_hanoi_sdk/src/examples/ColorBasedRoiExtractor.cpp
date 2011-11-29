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

#include "ColorBasedRoiExtractor.h"

namespace BRICS_3D {

ColorBasedRoiExtractor::ColorBasedRoiExtractor() {
	this->noOfFramesProcessed=0;
}

    ros::Publisher* ColorBasedRoiExtractor::getExtractedRegionPublisher() const
    {
        return extractedRegionPublisher;
    }

    void ColorBasedRoiExtractor::setExtractedRegionPublisher(ros::Publisher *extractedRegionPublisher)
    {
        this->extractedRegionPublisher = extractedRegionPublisher;
    }

ColorBasedRoiExtractor::~ColorBasedRoiExtractor() {
	// TODO Auto-generated destructor stub
	processingLogs->close();
	frameDelayLogs->close();
}


void ColorBasedRoiExtractor::initializeLimits(float minLimitH, float maxLimitH, float minLimitS, float maxLimitS){
	this->hsvBasedRoiExtractor.setMinH(minLimitH);
	this->hsvBasedRoiExtractor.setMaxH(maxLimitH);
	this->hsvBasedRoiExtractor.setMinS(minLimitS);
	this->hsvBasedRoiExtractor.setMaxS(maxLimitS);
	this->noOfFramesProcessed=0;
}

void ColorBasedRoiExtractor::kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud){



//	ROS_INFO("\ntransferred kinect_raw message successfully...... :)");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz_rgb_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hsv_extracted_roi_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());


    BRICS_3D::ColoredPointCloud3D *in_cloud = new BRICS_3D::ColoredPointCloud3D();
    BRICS_3D::ColoredPointCloud3D *extracted_cloud = new BRICS_3D::ColoredPointCloud3D();


    //Transform sensor_msgs::PointCloud2 msg to pcl::PointCloud
    pcl::fromROSMsg (cloud, *cloud_xyz_rgb_ptr);


	if(this->noOfFramesProcessed!=1 && this->noOfFramesProcessed<100 ){
			*frameDelayLogs << this->noOfFramesProcessed << " "<<  cloud_xyz_rgb_ptr->size()<< " " <<
					frameDelayTimer.elapsed()<< "\n";
			frameDelayTimer.restart();
	} else {
		frameDelayTimer.restart();
	}

    // cast PCL to BRICS_3D type
    pclTypeCaster.convertToBRICS3DDataType(cloud_xyz_rgb_ptr, in_cloud);
    ROS_INFO("Size of input cloud: %d ", in_cloud->getSize());

    //-------------------------------------------------------------------------------------------
	//perform HSV color based extraction
    //startProcessing=clock();
    processingTimer.restart();
	hsvBasedRoiExtractor.extractColorBasedROI(in_cloud, extracted_cloud);
	double processing_time = processingTimer.elapsed();
	//endProcessing=clock();
	ROS_INFO("Size of extracted cloud : %d ", extracted_cloud->getSize());

	if(noOfFramesProcessed<100)
	*processingLogs  << this->noOfFramesProcessed << " " << cloud_xyz_rgb_ptr->size() << " "<<
					processing_time << " " <<"\n";
	//-------------------------------------------------------------------------------------------

	//convert back to PCl format for publishing
	//pclTypeCaster.convertToPCLDataType(hsv_extracted_roi_ptr, &extracted_cloud);
	pclTypeCaster.convertToPCLDataType(hsv_extracted_roi_ptr, extracted_cloud);

	//setup frame_id of extracted cloud for publishing
	hsv_extracted_roi_ptr->header.frame_id = "/openni_rgb_optical_frame";

	//publish extracted region
	extractedRegionPublisher->publish(*hsv_extracted_roi_ptr);

		delete in_cloud;
		delete extracted_cloud;

//		ROS_INFO("no of frames processed %d",noOfFramesProcessed);

		this->noOfFramesProcessed++;

		if(noOfFramesProcessed>100){
			ROS_WARN("Logging Completed");
//			exit(0);
		}

}

}

