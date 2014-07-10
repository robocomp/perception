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

#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
/**
       \brief
       @author authorname
*/

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
	RoboCompRGBD::ColorSeq rgbMatrix;	
	RoboCompRGBD::depthType distanceMatrix;
	RoboCompRGBD::PointSeq points_kinect;
	RoboCompJointMotor::MotorStateMap h;
	RoboCompDifferentialRobot::TBaseState b;
	
	TagModelMap tagMap;
	QMutex *mutex;
	
	//PCL data structures
	pcl::PointCloud<PointT>::Ptr cloud;
	
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx, QObject *parent = 0);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void newAprilTag(const tagsList& tags);
	void doThePointClouds();
	void doTheAprilTags();
	void addTheTable(RoboCompInnerModelManager::Pose3D pose);
	void updateTable(RoboCompInnerModelManager::Pose3D pose);

public slots:
 	void compute(); 	
};

#endif