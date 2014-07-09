/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
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

SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)	
, cloud(new pcl::PointCloud<PointT>)
{
	
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}
void SpecificWorker::compute( )
{
	try
	{
		rgbd_proxy->getImage(rgbMatrix, distanceMatrix, points_kinect,  h, b);
	}
	catch(Ice::Exception e)
  {
		qDebug()<<"Error talking to rgbd_proxy: "<<e.what();
	}
	
	cloud->points.resize(points_kinect.size());
	
	for(unsigned int i=0; i<points_kinect.size(); i++)
	{
		cloud->points[i].x=points_kinect[i].x*1000; 
		cloud->points[i].y=points_kinect[i].y*1000; 
		cloud->points[i].z=points_kinect[i].z*1000; 
		cloud->points[i].r=rgbMatrix[i].red;
		cloud->points[i].g=rgbMatrix[i].green;
		cloud->points[i].b=rgbMatrix[i].blue;
	}
	
	//Do the PCL awesome processing here
	
	//Now show results
	RoboCompInnerModelManager::PointCloudVector pointcloud;
	pointcloud.resize(cloud->size());
  int j=0;
  for (pcl::PointCloud<PointT>::iterator it = cloud->points.begin (); it < cloud->points.end (); ++it)
  {
// 		pointcloud[j].r=it->r;
// 		pointcloud[j].g=it->g;
// 		pointcloud[j].b=it->b;
// 		pointcloud[j].x=(it->x);
// 		pointcloud[j].y=(it->y);
// 		pointcloud[j].z=(it->z);
		pointcloud[j].r=0;
		pointcloud[j].g=255;
		pointcloud[j].b=0;
 		pointcloud[j].x=points_kinect[j].x;
 		pointcloud[j].y=points_kinect[j].y;
 		pointcloud[j].z=points_kinect[j].z;

    
    j++;
  }
  
  try
  {
    innermodelmanager_proxy->setPointCloudData("cloud", pointcloud);
  }catch(Ice::Exception e)
  {
    qDebug()<<"Error talking to inermodelmanager_proxy: "<<e.what();
  }
		
	
}

void showPointCloud()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
};
void SpecificWorker::newAprilTag(const tagsList& tags)
{
	
}
