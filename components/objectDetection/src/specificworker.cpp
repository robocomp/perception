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
, mutex(new QMutex())
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
	doThePointClouds();
	
	drawThePointCloud(this->cloud);
	
	doTheAprilTags();
	
}

void SpecificWorker::doThePointClouds()
{
	//transform to pcl cloud
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
		cloud->points[i].x=points_kinect[i].x; 
		cloud->points[i].y=points_kinect[i].y; 
		cloud->points[i].z=points_kinect[i].z; 
		
		cloud->points[i].r=rgbMatrix[i].red;
		cloud->points[i].g=rgbMatrix[i].green;
		cloud->points[i].b=rgbMatrix[i].blue;
	}
	
	
	//Downsample the point cloud:
	
// 	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//   sor.setInputCloud (cloud);
//   sor.setLeafSize (0.01f, 0.01f, 0.01f);
//   sor.filter (*cloud);
  
}

void SpecificWorker::drawThePointCloud(pcl::PointCloud<PointT>::Ptr cloud)
{
	//Now show results
	RoboCompInnerModelManager::PointCloudVector pointcloud;
	pointcloud.resize(cloud->size());
  int j=0;
  for (pcl::PointCloud<PointT>::iterator it = cloud->points.begin (); it < cloud->points.end (); ++it)
  {
		
		pointcloud[j].r=it->r;
		pointcloud[j].g=it->g;
		pointcloud[j].b=it->b;
		pointcloud[j].x=(it->x);
		pointcloud[j].y=(it->y);
		pointcloud[j].z=(it->z);

    j++;
  }
  
  try
  {
    innermodelmanager_proxy->setPointCloudData("cloud", pointcloud);
  }
  catch(Ice::Exception e)
  {
    qDebug()<<"Error talking to inermodelmanager_proxy: "<<e.what();
  } 
}

void SpecificWorker::doTheAprilTags()
{
	mutex->lock();
	for (TagModelMap::iterator itMap=tagMap.begin();  itMap!=tagMap.end(); itMap++)
	{
		if (itMap->second.id == 3)
		{
			RoboCompInnerModelManager::Pose3D pose;
			pose.x = itMap->second.tx;
			pose.y = itMap->second.ty;
			pose.z = itMap->second.tz;
			pose.rx = itMap->second.rx;
			pose.ry = itMap->second.ry;
			pose.rz = itMap->second.rz;
			
			bool exists = false;
			
			RoboCompInnerModelManager::NodeInformationSequence node_sequence;
			innermodelmanager_proxy->getAllNodeInformation(node_sequence);
			for (unsigned int i=0; i<node_sequence.size(); i++)
			{
				if(node_sequence[i].id == "mesa")
				{
					exists = true;
					break;
				}
			}
			
			if(!exists)
			{
				std::cout<<"pintando"<<std::endl;
				addTheTable(pose);
			}
			else
				updateTable(pose);
			break;
 		}
	}
	mutex->unlock();
}

void SpecificWorker::addTheTable(RoboCompInnerModelManager::Pose3D pose)
{
	RoboCompInnerModelManager::Pose3D pose2;
	pose2.x = pose2.y = pose2.rx = pose2.ry = pose2.rz = pose2.z = 0;
	
	RoboCompInnerModelManager::meshType table_mesh;
	
 	table_mesh.meshPath = "/home/robocomp/robocomp/files/osgModels/basics/cubexxx.3ds";
	table_mesh.scaleX = 57.5;
	table_mesh.scaleY = 80; // <--- A 674mm radius table has a scale of "100"
	table_mesh.scaleZ = 57.5; 
	
	pose2.z = 57.5;
	pose2.y = 26;
	
	try
	{
		innermodelmanager_proxy->addTransform("mesa_T",  "static", "world", pose);
		innermodelmanager_proxy->addTransform("mesa_T2", "static", "mesa_T", pose2);
		innermodelmanager_proxy->addMesh("mesa", "mesa_T2", table_mesh);
	}
	catch(RoboCompInnerModelManager::InnerModelManagerError e)
	{
		std::cout<<e.text<<std::endl;
	}
	
}

void SpecificWorker::updateTable(RoboCompInnerModelManager::Pose3D pose)
{
	RoboCompInnerModelManager::Pose3D pose2;
	pose2.x = pose2.y = pose2.z = pose2.rx = pose2.ry = pose2.rz = 0;
	
	RoboCompInnerModelManager::meshType table_mesh;
	table_mesh.meshPath = "/home/robocomp/robocomp/files/osgModels/basics/cube.3ds";

 	innermodelmanager_proxy->setPoseFromParent("mesa_T", pose);

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
};
void SpecificWorker::newAprilTag(const tagsList& tags)
{
	mutex->lock();
	
	QTime c = QTime::currentTime();

	for (uint32_t i=0; i<tags.size(); i++)
	{
		if (tagMap.find(tags[i].id) != tagMap.end())
		{
			tagMap[tags[i].id] = AprilTagModel();
		}
		else
		{
			printf("new tag %d\n", tags[i].id);
		}

		tagMap[tags[i].id].id = tags[i].id;
		tagMap[tags[i].id].tx = tags[i].tx;
		tagMap[tags[i].id].ty = tags[i].ty;
		tagMap[tags[i].id].tz = tags[i].tz;
		tagMap[tags[i].id].rx = tags[i].rx;
		tagMap[tags[i].id].ry = tags[i].ry;
		tagMap[tags[i].id].rz = tags[i].rz;
		tagMap[tags[i].id].lastTime = c;
	}
	
	mutex->unlock();
}
