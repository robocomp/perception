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
#ifndef OBJECTDETECTION_H
#define OBJECTDETECTION_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <objectDetection.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompobjectDetection;

class objectDetectionI : public QObject , public virtual RoboCompobjectDetection::objectDetection
{
Q_OBJECT
public:
	objectDetectionI( GenericWorker *_worker, QObject *parent = 0 );
	~objectDetectionI();
	
	void grabTheAR(const Ice::Current&);
	void aprilFitModel(const string  &model, const Ice::Current&);
	void segmentImage(const Ice::Current&);
	void mindTheGapPC(const Ice::Current&);
	void getResult(const string  &image, const string  &pcd,  detectionResult  &detection, const Ice::Current&);
	void centroidBasedPose( float  &x,  float  &y,  float  &theta, const Ice::Current&);
	void reloadVFH(const Ice::Current&);
	void ransac(const string  &model, const Ice::Current&);
	void euclideanClustering( int  &numCluseters, const Ice::Current&);
	void passThrough(const Ice::Current&);
	void surfHomography( listType  &guesses, const Ice::Current&);
	void fitTheViewVFH(const Ice::Current&);
	void readThePointCloud(const string  &image, const string  &pcd, const Ice::Current&);
	void showObject(const int  numObject, const Ice::Current&);
	void convexHull(const string  &model, const Ice::Current&);
	void mirrorPC(const Ice::Current&);
	void statisticalOutliersRemoval(const Ice::Current&);
	void loadTrainedVFH(const Ice::Current&);
	void reset(const Ice::Current&);
	void normalSegmentation(const string  &model, const Ice::Current&);
	void getInliers(const string  &model, const Ice::Current&);
	void vfh( listType  &guesses, const Ice::Current&);
	void grabThePointCloud(const string  &image, const string  &pcd, const Ice::Current&);
	void fitModel(const string  &model, const string  &method, const Ice::Current&);
	void projectInliers(const string  &model, const Ice::Current&);
	void extractPolygon(const string  &model, const Ice::Current&);

	QMutex *mutex;
private:

	GenericWorker *worker;
public slots:


};

#endif
