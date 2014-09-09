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
	connect(ransac_button, SIGNAL(clicked()), this, SLOT(ransac_table()));
	connect(ec_button, SIGNAL(clicked()), this, SLOT(euclidean_clustering()));
	connect(list_clouds, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(showObject(QListWidgetItem*)));
	connect(reset_button, SIGNAL(clicked()), this, SLOT(reset()));
	
	
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

void SpecificWorker::ransac_table()
{
	std::cout<<"About to send RANSAC"<<std::endl;
	objectdetection_proxy->ransac("table");
	std::cout<<"sent RANSAC"<<std::endl;
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
void SpecificWorker::euclidean_clustering()
{
	int num_clusters;
	objectdetection_proxy->euclideanClustering(num_clusters);
	list_clouds->clear();
	stringstream ss;
	for(int i=0; i<num_clusters; i++)
	{
		ss << i;
		string name = "cloud_" + ss.str();
		ss.str("");
		list_clouds->addItem(QString::fromStdString(name));
	}
		
	std::cout<<num_clusters<<std::endl;
}

void SpecificWorker::showObject(QListWidgetItem *item)
{
	QString number = item->text();
	number.remove("cloud_");
	int num_object = number.toInt();
	
	objectdetection_proxy->showObject(num_object);
}

void SpecificWorker::reset()
{
	objectdetection_proxy->reset();
}

void SpecificWorker::compute( )
{
}
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
};
