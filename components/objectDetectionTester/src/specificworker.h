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

#include <QStringList>
#include <genericworker.h>
/**
       \brief
       @author authorname
*/

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx, QObject *parent = 0);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	
public slots:
 	void compute(); 	
	
	void mirror();
	void mindTheGap();
	
	void fitPrismNaive();
	void fitPrismMCMC();
	void fitPrismPf();
	void fitCylinder();
	
	void aprilFitTable();
	void aprilFitBox();
	
	void reloadVFH();
	void loadTrainedVFH();
	void vfh();
	void surfHomography();
	
	void getInliers();
	void projectInliers();
	void convexHull();
	void extractPolygon();
	void ransac_table();
	void euclidean_clustering();
	void showObject(QListWidgetItem *item);
	void reset();
	void normal_segmentation();
};

#endif