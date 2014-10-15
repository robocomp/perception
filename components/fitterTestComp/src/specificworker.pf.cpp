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

SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
	final_ = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
	first = true;
	new_cloud_available_flag = false;

	vvv = boost::shared_ptr<Viewer>(new Viewer("cubeCloud.xml", aqui));
	boost::shared_ptr<RectPrism> shape(new RectPrism());
	timer.start(1);
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

void SpecificWorker::translateClouds(pcl::PointCloud<PointT>::Ptr c_dest,  const pcl::PointCloud<PointT>::ConstPtr &c_org )
{
	pcl::PointCloud<PointT>::Ptr cloud_clean (new pcl::PointCloud<PointT>);
	pcl::PointCloud<int> sampled_indices;
	pcl::UniformSampling<PointT> uniform_sampling;
	uniform_sampling.setInputCloud (c_org);
	uniform_sampling.setRadiusSearch (0.05f);
	uniform_sampling.compute (sampled_indices);
	pcl::copyPointCloud (*c_org, sampled_indices.points, *cloud_clean);

	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	pcl::SampleConsensusModelPlane<PointT>::Ptr
		model_s(new pcl::SampleConsensusModelPlane<PointT> (cloud_clean));
	//Ransac
	pcl::RandomSampleConsensus<PointT> ransac (model_s);
	ransac.setDistanceThreshold (0.05);
	ransac.computeModel();
	ransac.getInliers(inliers->indices);

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (cloud_clean);
	extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter(*cloud_clean);
	std::vector< int > nanindexes;
	pcl::removeNaNFromPointCloud(*cloud_clean,*cloud_clean,nanindexes);

		//cluster extraction
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (cloud_clean);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (0.1);
	ec.setMinClusterSize (30);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_clean);
	ec.extract (cluster_indices);

	//Get the biggest
	pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
	for (std::vector<int>::const_iterator pit = cluster_indices.begin()->indices.begin (); pit != cluster_indices.begin()->indices.end (); pit++)
	{
		cloud_cluster->points.push_back (cloud_clean->points[*pit]);
	}
  

	c_dest->clear();
	for(pcl::PointCloud<PointT>::const_iterator it = cloud_cluster->begin(); it != cloud_cluster->end(); it++)
	{
		PointT p;
		p.x=it->x*1000;
		p.y=it->y*1000;
		p.z=it->z*1000;
		p.r=it->r;
		p.g=it->g;
		p.b=it->b;
		c_dest->push_back(p);
	}

	//cout<<"Size: "<<c_dest->size()<<endl;
}

void SpecificWorker::fit_cb(RectPrism  shape)
{
	static int tick = 0;

	printf("\n------ Tick %d\n", tick++);
	vvv->setPose("cube_0_t", shape.getCenter(), shape.getRotation(), shape.getWidth() );
	vvv->setScale("cube_0", shape.getWidth()(0)/2, shape.getWidth()(1)/2, shape.getWidth()(2)/2);
	//check this for inconsistency since transofrm might be touching this cloud
	if (new_cloud_available_flag==true)
	{
		//fitter->setPointCloud(final_);
		new_cloud_available_flag=false;
	}

	cout<<"Best: "<<fitter->getBestParticle().getWeight()<<endl;
	printf("  T = (%f, %f, %f)\n", shape.getCenter()(0), shape.getCenter()(1), shape.getCenter()(2));
	printf("  R = (%f, %f, %f)\n", shape.getRotation()(0), shape.getRotation()(1), shape.getRotation()(2));
	printf("  S = (%f, %f, %f)\n", shape.getWidth()(0), shape.getWidth()(1), shape.getWidth()(2));
	printf("  vars: T:%f   R:%f   size:%f\n", fitter->getBestParticle().varianceC(0), fitter->getBestParticle().varianceR(0), fitter->getBestParticle().varianceW(0));

	printf("\n");
}

void SpecificWorker::compute( )
{
	cloud->clear();
	try
	{
		rgbd_proxy->getImage(rgbMatrix, distanceMatrix, points_kinect,  h, b);
		//cout<<"Got: "<<points_kinect.size()<<endl;
		for(uint32_t i=0; i<points_kinect.size(); i++)
		{
		// cout<<points_kinect[i].z<<endl;
		if(points_kinect[i].z<=10000)
		{
			PointT point;
			point.x=points_kinect[i].x/1000.f;
			point.y=points_kinect[i].y/1000.f;
			point.z=points_kinect[i].z/1000.f;
			point.r=rgbMatrix[i].red;
			point.g=rgbMatrix[i].green;
			point.b=rgbMatrix[i].blue;

			cloud->push_back(point);
		}
		}
		//new_cloud_available_flag=true;
	}
	catch(Ice::Exception e)
	{
		qDebug()<<"Error talking to rgbd_proxy: "<<e.what();
	}

	translateClouds(final_, cloud);

	vvv->setPointCloud(final_);

	vvv->showImage(640,480, (uint8_t *)&rgbMatrix[0]);

	vvv->update();

	if (first)
	{
		first=false;
		fitter = new PfRectPrismFitting(200,  final_ );
		boost::function<void (RectPrism)> f = boost::bind (&SpecificWorker::fit_cb, this, _1);
		fitter->registerCallback (f);
		fitter->start ();
	}

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
};
