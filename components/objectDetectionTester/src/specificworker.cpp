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
	connect(get_inliers_button, SIGNAL(clicked()), this, SLOT(getInliers()));
	connect(project_inliers_button, SIGNAL(clicked()), this, SLOT(projectInliers()));
	connect(convex_hull_button, SIGNAL(clicked()), this, SLOT(convexHull()));
 	connect(extract_polygon_button, SIGNAL(clicked()), this, SLOT(extractPolygon()));
	connect(ransac_button, SIGNAL(clicked()), this, SLOT(ransac_table()));
	connect(ec_button, SIGNAL(clicked()), this, SLOT(euclidean_clustering()));
	connect(list_clouds, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(showObject(QListWidgetItem*)));
	connect(reset_button, SIGNAL(clicked()), this, SLOT(reset()));
	connect(normal_segmentation_button, SIGNAL(clicked()), this, SLOT(normal_segmentation()));
	connect(grab_pc_button, SIGNAL(clicked()), this, SLOT(grab_pc()));
	connect(pass_through_button, SIGNAL(clicked()), this, SLOT(passThrough()));
	connect(statistical_outliers_removal_button, SIGNAL(clicked()), this, SLOT(passThrough()));
	connect(centroid_based_pose_button, SIGNAL(clicked()), this, SLOT(centroidBasedPose()));
	
	//aspect recognition
	connect(reload_vfh_button, SIGNAL(clicked()), this, SLOT(reloadVFH()));
	connect(load_trained_vfh_button, SIGNAL(clicked()), this, SLOT(loadTrainedVFH()));
	connect(vfh_button, SIGNAL(clicked()), this, SLOT(vfh()));
	connect(SURF_button, SIGNAL(clicked()), this, SLOT(surfHomography()));
	//vfh fit
	connect(fit_the_view_button, SIGNAL(clicked()), this, SLOT(fitTheViewVFH()));
	
	//april fits
	connect(april_fit_table_button, SIGNAL(clicked()), this, SLOT(aprilFitTable()));
	connect(april_fit_box_button, SIGNAL(clicked()), this, SLOT(aprilFitBox()));
	
	//fitters
	connect(fit_prism_naive_button, SIGNAL(clicked()), this, SLOT(fitPrismNaive()));
	connect(fit_prism_mcmc_button, SIGNAL(clicked()), this, SLOT(fitPrismMCMC()));
	connect(fit_prism_pf_button, SIGNAL(clicked()), this, SLOT(fitPrismPf()));
	connect(fit_cylinder_button, SIGNAL(clicked()), this, SLOT(fitCylinder()));
	
	//mirror
	connect(mirror_pc_button, SIGNAL(clicked()), this, SLOT(mirror()));
	connect(mind_the_gap_button, SIGNAL(clicked()), this, SLOT(mindTheGap()));
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

void SpecificWorker::grab_pc()
{
	objectdetection_proxy->grabThePointCloud();
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

void SpecificWorker::mirror()
{
	objectdetection_proxy->mirrorPC();
}

void SpecificWorker::mindTheGap()
{
	objectdetection_proxy->mindTheGapPC();
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

void SpecificWorker::normal_segmentation()
{
	objectdetection_proxy->normalSegmentation("table");
}

void SpecificWorker::fitPrismNaive()
{
	objectdetection_proxy->fitModel("prism", "naive");
}

void SpecificWorker::fitPrismMCMC()
{
	objectdetection_proxy->fitModel("prism", "mcmc");
}

void SpecificWorker::fitPrismPf()
{
	objectdetection_proxy->fitModel("prism", "pf");
}


void SpecificWorker::fitCylinder()
{
	objectdetection_proxy->fitModel("cylinder", "naive");
}

void SpecificWorker::aprilFitTable()
{
	objectdetection_proxy->aprilFitModel("table");
}

void SpecificWorker::aprilFitBox()
{
	objectdetection_proxy->aprilFitModel("box");
}

void SpecificWorker::fitTheViewVFH()
{
	objectdetection_proxy->fitTheViewVFH();
}

void SpecificWorker::centroidBasedPose()
{
	objectdetection_proxy->centroidBasedPose();
}

void SpecificWorker::vfh()
{
	std::vector<string> guesses;
	objectdetection_proxy->vfh(guesses);
	QStringList pieces;
	QString path_to_pcd, name_of_object;
	vfh_listView->clear();
	
	for(int i = 0; i < guesses.size(); i++)
	{
		path_to_pcd = QString::fromStdString(guesses[i]);
		pieces = path_to_pcd.split( "/" );
		name_of_object = pieces.at( pieces.length() - 2 );
		vfh_listView->addItem(name_of_object);
	}
}

void SpecificWorker::statisticalOutliersRemoval()
{
	objectdetection_proxy->statisticalOutliersRemoval();
}

void SpecificWorker::passThrough()
{
	objectdetection_proxy->passThrough();
}

void SpecificWorker::surfHomography()
{
	std::vector<string> guesses;
	objectdetection_proxy->surfHomography(guesses);
}

void SpecificWorker::euclidean_clustering()
{
	int num_clusters;
	objectdetection_proxy->euclideanClustering(num_clusters);
	std::cout<<"euclidean_clustering done!"<<std::endl;
	list_clouds->clear();
	stringstream ss;
	for(int i=0; i<num_clusters; i++)
	{
		ss << i;
		string name = "cloud_" + ss.str();
		ss.str("");
		std::cout<<ss<<std::endl;
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

void SpecificWorker::reloadVFH()
{
	objectdetection_proxy->reloadVFH();
}

void SpecificWorker::loadTrainedVFH()
{
	objectdetection_proxy->loadTrainedVFH();
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
