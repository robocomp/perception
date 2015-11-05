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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
,cloud(new pcl::PointCloud<PointT>)
,ransac_inliers (new pcl::PointIndices)
,rgb_image(480,640, CV_8UC3, cv::Scalar::all(0))
,color_segmented(480,640, CV_8UC3, cv::Scalar::all(0))
,table(new Table())
{
	//let's set the sizes
	table->set_board_size(500,30,500);
	
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{



	
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
  
}


void SpecificWorker::grabTheAR()
{

}

void SpecificWorker::aprilFitModel(const string &model)
{

}

void SpecificWorker::segmentImage()
{
	#ifdef DEBUG
	cv::imwrite("nosegmentada.png",rgb_image);
	std::cout<<"setting image"<<std::endl;
	#endif
	
	segmentator.set_image(&rgb_image);
	
	#ifdef DEBUG
	std::cout<<"Segmenting image"<<std::endl;
	#endif
	
	
	segmentator.set_tresholds(100, 150);
	color_segmented = segmentator.segment();
	
	#ifdef DEBUG
	std::cout<<"Segmented"<<std::endl;
	cv::imwrite("Segmentada.png",color_segmented);
	cv::Mat yellow, pink, green;
	cv::inRange(color_segmented, cv::Scalar(0, 150, 150), cv::Scalar(80, 255, 255), yellow);
	cv::imwrite("yellow.png",yellow);
	cv::inRange(color_segmented, cv::Scalar(65, 15, 125), cv::Scalar(150, 100, 255), pink);
	cv::imwrite("pink.png",pink);
	cv::inRange(color_segmented, cv::Scalar(25, 75, 50), cv::Scalar(106, 255, 150), green);
	cv::imwrite("green.png",green);
	#endif
}

void SpecificWorker::mindTheGapPC()
{

}

string SpecificWorker::getResult(const string &image, const string &pcd)
{

}

void SpecificWorker::centroidBasedPose(float &x, float &y, float &theta)
{

}

void SpecificWorker::reloadVFH()
{

}

void SpecificWorker::ransac(const string &model)
{
	table->fit_board_with_RANSAC( cloud, ransac_inliers, 15);
}

void SpecificWorker::euclideanClustering(int &numCluseters)
{

}

void SpecificWorker::passThrough()
{

}

void SpecificWorker::surfHomography(listType &guesses)
{

}

void SpecificWorker::fitTheViewVFH()
{

}

void SpecificWorker::showObject(const int numObject)
{

}

void SpecificWorker::convexHull(const string &model)
{

}

void SpecificWorker::mirrorPC()
{

}

void SpecificWorker::statisticalOutliersRemoval()
{

}

void SpecificWorker::loadTrainedVFH()
{

}

void SpecificWorker::reset()
{

}

void SpecificWorker::normalSegmentation(const string &model)
{

}

void SpecificWorker::getInliers(const string &model)
{

}

void SpecificWorker::vfh(listType &guesses)
{

}

void SpecificWorker::grabThePointCloud(const string &image, const string &pcd)
{
	try
	{
	rgbd_proxy->getImage(rgbMatrix, distanceMatrix, points_kinect,  h, b);
	}
	catch(Ice::Exception e)
	{
	qDebug()<<"Error talking to rgbd_proxy: "<<e.what();
	}

	#ifdef DEBUG
		cout<<"SpecificWorker::grabThePointcloud rgbMatrix.size(): "<<rgbMatrix.size()<<endl;
	#endif
	
	for(unsigned int i=0; i<rgbMatrix.size(); i++)
	{
		int row = i/640;
		int column = i-(row*640);
		
		rgb_image.at<cv::Vec3b>(row,column) = cv::Vec3b(rgbMatrix[i].blue, rgbMatrix[i].green, rgbMatrix[i].red);
	}
	#ifdef DEBUG
		cv::imwrite( "rgb.png", rgb_image);
	#endif
	
		
	cloud->points.resize(points_kinect.size());
	for (unsigned int i=0; i<points_kinect.size(); i++)
	{
		memcpy(&cloud->points[i], &points_kinect[i],3*sizeof(float));
		cloud->points[i].r=rgbMatrix[i].red;
		cloud->points[i].g=rgbMatrix[i].green;
		cloud->points[i].b=rgbMatrix[i].blue;
	}
	cloud->width = 1;
	cloud->height = points_kinect.size();
	
	std::vector< int > index;
	removeNaNFromPointCloud (*cloud, *cloud, index);
	
	timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	string pcdname =  "/home/robocomp/robocomp/components/perception/" + QString::number(ts.tv_sec).toStdString() + ".pcd";
	printf("<%s>\n", pcdname.c_str());
	writer.write<PointT> ( pcdname, *cloud, false);
  
}

void SpecificWorker::fitModel(const string &model, const string &method)
{

}

void SpecificWorker::setContinousMode(const bool &mode)
{

}

void SpecificWorker::projectInliers(const string &model)
{

}

void SpecificWorker::extractPolygon(const string &model)
{

}

void SpecificWorker::newAprilTag(const tagsList &tags)
{

}






