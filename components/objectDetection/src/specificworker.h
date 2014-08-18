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
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <qmat/QMatAll>

#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <innermodel/innermodel.h>

/**
       \brief
       @author authorname
*/

typedef struct TableSize
{
	static const int x=1000;
	static const int y=10;
	static const int z=300; 
};

class AprilTagModel
{
public:
	int id;
	float tx;
	float ty;
	float tz;
	float rx;
	float ry;
	float rz;
	QTime lastTime;
};

typedef pcl::PointXYZRGB PointT;
typedef std::map<int, AprilTagModel> TagModelMap;

class SpecificWorker : public GenericWorker
{
	//table related stuff
	bool removeTheTable;
	TableSize tablesize;
	
	RoboCompRGBD::ColorSeq rgbMatrix;	
	RoboCompRGBD::depthType distanceMatrix;
	RoboCompRGBD::PointSeq points_kinect;
	RoboCompJointMotor::MotorStateMap h;
	RoboCompDifferentialRobot::TBaseState b;
	
	TagModelMap tagMap;
	QMutex *mutex;
	QMutex *point_cloud_mutex;
	
	// Create the filtering object
	pcl::PassThrough<PointT> pass;
	
	//PCL data structures
	pcl::PointCloud<PointT>::Ptr cloud;
	pcl::PointCloud<PointT>::Ptr segmented_cloud;
	pcl::PointCloud<PointT>::Ptr downsampled_cloud;
	
	InnerModel *innermodel;
	
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	void setModel2Fit(const string& model);
	void removePCwithinModel(const string& model);
	void removeTablePC(const string& model);
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void newAprilTag(const tagsList& tags);
	
	void doThePointClouds();
	void doTheAprilTags();
	
	void doTheBox();
	void addTheBox(RoboCompInnerModelManager::Pose3D pose);
	void updateTheBox(RoboCompInnerModelManager::Pose3D pose);
	
	void doTheTable();
	void addTheTable(RoboCompInnerModelManager::Pose3D pose);
	void updateTheTable(RoboCompInnerModelManager::Pose3D pose);
	
	void drawThePointCloud(pcl::PointCloud<PointT>::Ptr cloud);
	
private:
	bool add_point_cloud_to_innermodels(const std::string &id, const RoboCompInnerModelManager::PointCloudVector &cloud);
	bool add_tranform_to_innermodels(const std::string &item, const std::string &engine, const std::string &base, const RoboCompInnerModelManager::Pose3D &pose);
	bool add_mesh_to_innermodels(const std::string &item, const std::string &base, const RoboCompInnerModelManager::meshType &m);
	void update_transforms_on_innermodels (const std::string &item, const RoboCompInnerModelManager::Pose3D pose);
	
public slots:
 	void compute(); 

};

#endif