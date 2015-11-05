/*
 *    Copyright (C) 2015 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
       \brief
       @author authorname
*/







#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <genericworker.h>
#include <innermodel/innermodel.h>

#include "color_segmentation/Segmentator.h"
#include "shapes/table.h"

#define DEBUG 1
typedef pcl::PointXYZRGB PointT;


class SpecificWorker : public GenericWorker
{
	
	InnerModel *innermodel;
	
	pcl::PCDWriter writer;
	
	//Cloud of the current points for pcl
	pcl::PointCloud<PointT>::Ptr cloud;
	pcl::PointIndices::Ptr ransac_inliers;
	
	//Image of the current view for opencv
	cv::Mat rgb_image;
	cv::Mat color_segmented;
	
	//Point cloud grabing
	RoboCompRGBD::ColorSeq rgbMatrix;	
	RoboCompRGBD::depthType distanceMatrix;
	RoboCompRGBD::PointSeq points_kinect;
	RoboCompJointMotor::MotorStateMap h;
	RoboCompDifferentialRobot::TBaseState b;
	
	//color Segmentator
 	Segmentator segmentator;
	
	boost::shared_ptr<Table> table;
  
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void grabThePointCloud(const string &image, const string &pcd);
	void segmentImage();
	
	
	void grabTheAR();
	void aprilFitModel(const string &model);
	void mindTheGapPC();
	string getResult(const string &image, const string &pcd);
	void centroidBasedPose(float &x, float &y, float &theta);
	void reloadVFH();
	void ransac(const string &model);
	void euclideanClustering(int &numCluseters);
	void passThrough();
	void surfHomography(listType &guesses);
	void fitTheViewVFH();
	void showObject(const int numObject);
	void convexHull(const string &model);
	void mirrorPC();
	void statisticalOutliersRemoval();
	void loadTrainedVFH();
	void reset();
	void normalSegmentation(const string &model);
	void getInliers(const string &model);
	void vfh(listType &guesses);
	void fitModel(const string &model, const string &method);
	void setContinousMode(const bool &mode);
	void projectInliers(const string &model);
	void extractPolygon(const string &model);
	void newAprilTag(const tagsList &tags);

public slots:
	void compute(); 	

private:
	
};

#endif

