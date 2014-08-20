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

SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx)	
{
	connect(fit_table_button, SIGNAL(clicked()), this, SLOT(fitTable()));
	connect(get_inliers_button, SIGNAL(clicked()), this, SLOT(getInliers()));
	connect(project_inliers_button, SIGNAL(clicked()), this, SLOT(projectInliers()));
	connect(convex_hull_button, SIGNAL(clicked()), this, SLOT(convexHull()));
 	connect(extract_polygon_button, SIGNAL(clicked()), this, SLOT(extractPolygon()));
	
	connect(fit_box_button, SIGNAL(clicked()), this, SLOT(fitBox()));
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

void SpecificWorker::fitTable()
{
	objectdetection_proxy->setModel2Fit("table");
}

void SpecificWorker::getInliers()
{
	objectdetection_proxy->getInliers("table");
}

void SpecificWorker::projectInliers()
{
	objectdetection_proxy->projectInliers("table");
}

void SpecificWorker::convexHull()
{
	objectdetection_proxy->convexHull("table");
}

void SpecificWorker::extractPolygon()
{
 	objectdetection_proxy->extractPolygon("table");
}

void SpecificWorker::fitBox()
{
	objectdetection_proxy->setModel2Fit("box");
}

void SpecificWorker::compute( )
{
}
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
};