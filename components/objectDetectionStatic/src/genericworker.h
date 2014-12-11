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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

// #include <ipp.h>
#include "config.h"
#include <QtGui>
#include <stdint.h>
#include <qlog/qlog.h>
#include <CommonBehavior.h>
#include <InnerModelManager.h>
#include <RGBD.h>
#include <objectDetection.h>
#include <AprilTags.h>

#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

typedef map <string,::IceProxy::Ice::Object*> MapPrx;

using namespace std;

/**
       \brief
       @author authorname
*/
using namespace RoboCompInnerModelManager;
using namespace RoboCompRGBD;
using namespace RoboCompobjectDetection;
using namespace RoboCompAprilTags;

class GenericWorker : public QObject
{
Q_OBJECT
public:
	GenericWorker(MapPrx& mprx, QObject *parent = 0);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);
	
	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;                //Shared mutex with servant

	InnerModelManagerPrx innermodelmanager_proxy;
	RGBDPrx rgbd_proxy;
	virtual void  statisticalOutliersRemoval() = 0;
	virtual void  passThrough() = 0;
	virtual void  grabThePointCloud(const string& image, const string& pcd) = 0;
	virtual void  aprilFitModel(const string& model) = 0;
	virtual void  fitModel(const string& model, const string& method) = 0;
	virtual void  getInliers(const string& model) = 0;
	virtual void  projectInliers(const string& model) = 0;
	virtual void  convexHull(const string& model) = 0;
	virtual void  extractPolygon(const string& model) = 0;
	virtual void  ransac(const string& model) = 0;
	virtual void  normalSegmentation(const string& model) = 0;
	virtual void  euclideanClustering(Ice::Int& numCluseters) = 0;
	virtual void  showObject(int numObject) = 0;
	virtual void  reset() = 0;
	virtual void  mirrorPC() = 0;
	virtual void  mindTheGapPC() = 0;
	virtual void  reloadVFH() = 0;
	virtual void  loadTrainedVFH() = 0;
	virtual void  fitTheViewVFH() = 0;
	virtual void  vfh(listType& guesses) = 0;
	virtual void  surfHomography(listType& guesses) = 0;
	virtual void  centroidBasedPose(Ice::Float& x, Ice::Float& y, Ice::Float& theta) = 0;
	virtual void  segmentImage() = 0;
	virtual void  grabTheAR() = 0;
	virtual string getResult(const string& image, const string& pcd) = 0;
	virtual void  setContinousMode(bool mode) = 0;
	virtual void  newAprilTag(const tagsList& tags) = 0;

protected:
	QTimer timer;
	int Period;
public slots:
	virtual void compute() = 0;
signals:
	void kill();
};

#endif