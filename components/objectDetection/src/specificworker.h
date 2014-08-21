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
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/pcd_io.h>
#include <innermodel/innermodel.h>

#include "shapes/table.h"


/**
       \brief
       @author authorname
*/

typedef struct TableSize
{
	static const int x=1000;
	static const int y=20;
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
	boost::shared_ptr<Table> table;
	TableSize tablesize;
	PointT table_offset;
	
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
	pcl::PointCloud<PointT>::Ptr original_cloud;
	pcl::PointCloud<PointT>::Ptr segmented_cloud;
	pcl::PointCloud<PointT>::Ptr downsampled_cloud;
	
	//table data
	pcl::PointIndices::Ptr model_inliers_indices;
	pcl::PointCloud<PointT>::Ptr plane_hull;
	pcl::PointCloud<PointT>::Ptr cloud_hull;
	
	//action flags
	bool getTableInliers_flag, projectTableInliers_flag, tableConvexHull_flag, extractTablePolygon_flag;
	
	InnerModel *innermodel;
	
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	
	//generic functions
	void setModel2Fit(const string& model);
	void getInliers(const string& model);
	void projectInliers(const string& model);
	void convexHull(const string& model);
	void extractPolygon(const string& model);
	
	//table specific functions
	void getTableInliers();
	void projectTableInliers();
	void tableConvexHull();
	void extractTablePolygon();
	
	//utils
	static void threePointsToPlane (const PointT &point_a, 
                            const PointT &point_b, 
                            const PointT &point_c, 
                            const pcl::ModelCoefficients::Ptr plane);
	
	
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