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

SpecificWorker::SpecificWorker(MapPrx& mprx) : table(new Table()), box(new RectPrism()),
GenericWorker(mprx), mutex(new QMutex()), point_cloud_mutex(new QMutex()), cloud(new pcl::PointCloud<PointT>), original_cloud(new pcl::PointCloud<PointT>),
segmented_cloud(new pcl::PointCloud<PointT>), model_inliers_indices(new pcl::PointIndices), plane_hull(new pcl::PointCloud<PointT>), cloud_hull(new pcl::PointCloud<PointT>), 
euclidean_mutex(new QMutex())

{
	innermodel = new InnerModel("/home/robocomp/robocomp/components/perception/etc/genericPointCloud.xml");
	
	//let's set the sizes
	table->set_board_size(1000,10,300);
	box->set_size(QVec::vec3(57.5, 80.0, 57.5));

	//let's set the ofsets
	table_offset.x = -150;
	table_offset.y = 0;
	table_offset.z = 300;
	
	box_offset.x = 0;
	box_offset.y = 65.5;
	box_offset.z = 26;
	
	//action flags
	getTableInliers_flag = projectTableInliers_flag = tableConvexHull_flag = extractTablePolygon_flag = getTableRANSAC_flag = euclideanClustering_flag = showOnlyObject_flag = false;
	
	//only released when the euclidean cluster is performed
	euclidean_mutex->lock();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

void SpecificWorker::setModel2Fit(const string& model)
{
	if(model=="box")
	{
		fitTheBox();
	}
	if(model=="table")
	{
		fitTheTable();
	}
}

void SpecificWorker::getInliers(const string& model)
{
	if(model=="table")
		getTableInliers_flag = !getTableInliers_flag;
}

void SpecificWorker::ransac(const string& model)
{
	if(model=="table")
		getTableRANSAC_flag = !getTableRANSAC_flag;

}

void SpecificWorker::projectInliers(const string& model)
{
	if(model=="table")
		projectTableInliers_flag = !projectTableInliers_flag;
}

void SpecificWorker::convexHull(const string& model)
{
	if(model=="table")
		tableConvexHull_flag = !tableConvexHull_flag;
}

//it only works for the table now
void SpecificWorker::extractPolygon(const string& model)
{
	if(model=="table")
	{
		extractTablePolygon_flag = ! extractTablePolygon_flag;
	}
}

void SpecificWorker::euclideanClustering(int &num_clusters)
{
	//release flag to perferorm euclidean clustering
	euclideanClustering_flag = ! euclideanClustering_flag;
	
	//wait for euclidean clustering to be performed
	euclidean_mutex->lock();
	num_clusters = cluster_indices.size();
}

void SpecificWorker::compute( )
{
		
		updatePointCloud();
		
		if(getTableInliers_flag)
		{
			table->get_table_inliers(cloud, cloud, model_inliers_indices);
		}
		
		if(projectTableInliers_flag)
		{
			table->project_board_inliers(cloud, model_inliers_indices, plane_hull);
			//update showing cloud
			*this->cloud = *plane_hull;
		}
		
		if(getTableRANSAC_flag)
		{
			QVec rotation = table->get_board_rotation();
// 			rotation.print("PREROTATION: ");
			table->fit_board_with_RANSAC( cloud, 0.01);
			drawTheTable();
		}
		
		if(tableConvexHull_flag)
		{
			table->board_convex_hull(plane_hull, cloud_hull);
			
			this->cloud = cloud_hull;
		}
		
		if(extractTablePolygon_flag)
		{
			RTMat viewpoint_transform = innermodel->getTransformationMatrix("robot", "rgbd_t");
			table->extract_table_polygon(this->original_cloud, cloud_hull, QVec::vec3(viewpoint_transform(0,3), viewpoint_transform(1,3), viewpoint_transform(2,3)) , 20, 1500, this->cloud);
		}
		
		if(euclideanClustering_flag)
		{
			performEuclideanClustering();
			euclideanClustering_flag = ! euclideanClustering_flag;
		}
		
		if(showOnlyObject_flag)
		{
			drawThePointCloud(cluster_clouds[object_to_show]);
		}
		else
			drawThePointCloud(this->cloud);
	
}
void SpecificWorker::showObject(int object_to_show)
{
	this->object_to_show = object_to_show;
	showOnlyObject_flag = true;
}

void SpecificWorker::reset()
{
	//action flags
	getTableInliers_flag = projectTableInliers_flag = tableConvexHull_flag = extractTablePolygon_flag = getTableRANSAC_flag = euclideanClustering_flag = showOnlyObject_flag = false;
}

void SpecificWorker::fitTheBox()
{
	//put the box processing here:
	mutex->lock();
	for (TagModelMap::iterator itMap=tagMap.begin();  itMap!=tagMap.end(); itMap++)
	{
		//move the tQVec v = r * QVec::vec3(p.x, p.y, p.z);
		
		if (itMap->second.id == 3)
		{
// 			RoboCompInnerModelManager::Matrix m = innermodelmanager_proxy->getTransformationMatrix("rgbd_t", "robot");
// 			
// 			QMat PP = QMat(m.rows, m.cols);
// 			for (int r=0; r<m.rows; r++)
// 			{
// 				for (int c=0; c<m.cols; c++)
// 				{
// 					PP(r,c) = m.data[r*m.cols+c];
// 				}
// 			}
			QMat PP = innermodel->getTransformationMatrix("robot", "rgbd_t");
			
			RTMat object_tr( itMap->second.rx, itMap->second.ry, itMap->second.rz, itMap->second.tx, itMap->second.ty, itMap->second.tz );

							 
			const RTMat translated_obj = PP * object_tr;
			
			const QVec tr = translated_obj.getTr();
			const QVec r = translated_obj.extractAnglesR_min();

			
			RoboCompInnerModelManager::Pose3D pose;
			pose.x = translated_obj(0,3);
			pose.y = translated_obj(1,3);
			pose.z = translated_obj(2,3);
			pose.rx = r(0);
			pose.ry = r(1);
			pose.rz = r(2);
			
			bool exists = false;
			
			RoboCompInnerModelManager::NodeInformationSequence node_sequence;
			innermodelmanager_proxy->getAllNodeInformation(node_sequence);
			for (unsigned int i=0; i<node_sequence.size(); i++)
			{
				if(node_sequence[i].id == "box")
				{
					exists = true;
					break;
				}
			}
			
			if(!exists)
			{
				std::cout<<"pintando"<<std::endl;
				addTheBox(pose);
			}
			else
				updateTheBox(pose);
			break;
 		}
	}
	mutex->unlock();
	
}

void SpecificWorker::drawTheTable()
{
	const QVec r = table->get_board_rotation();
	const QVec t = table->get_board_center();
	
	RoboCompInnerModelManager::Pose3D pose;
	pose.x = t(0);
	pose.y = t(1);
	pose.z = t(2);
	pose.rx = r(0);
	pose.ry = r(1);
	pose.rz = r(2);
	
	innermodelmanager_proxy->setPose("robot", "table_T2", pose);
	innermodel->updateTransformValues("table_T2", pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz, "robot");
	
// 		bool exists = false;
// 	
// 	RoboCompInnerModelManager::NodeInformationSequence node_sequence;
// 	innermodelmanager_proxy->getAllNodeInformation(node_sequence);
// 	for (unsigned int i=0; i<node_sequence.size(); i++)
// 	{
// 		if(node_sequence[i].id == "table")
// 		{
// 			exists = true;
// 			break;
// 		}
// 	}
// 	
// 	if(!exists)
// 		addTheTable(pose);
// 	else
// 		updateTheTable(pose);
	
	
// 	bool exists = false;
// 	
// 	RoboCompInnerModelManager::NodeInformationSequence node_sequence;
// 	innermodelmanager_proxy->getAllNodeInformation(node_sequence);
// 	for (unsigned int i=0; i<node_sequence.size(); i++)
// 	{
// 		if(node_sequence[i].id == "table")
// 		{
// 			exists = true;
// 			break;
// 		}
// 	}
// 	
// 	if(!exists)
// 		addTheTable(pose);
// 	else
// 		updateTheTable(pose);
}

void SpecificWorker::fitTheTable()
{
	//put the box processing here:
	mutex->lock();
	for (TagModelMap::iterator itMap=tagMap.begin();  itMap!=tagMap.end(); itMap++)
	{
		//move the tags to the robot reference frame
		
		if (itMap->second.id == 0)
		{
// 			RoboCompInnerModelManager::Matrix m = innermodelmanager_proxy->getTransformationMatrix("rgbd_t", "robot");
// 			
// 			QMat PP = QMat(m.rows, m.cols);
// 			for (int r=0; r<m.rows; r++)
// 			{
// 				for (int c=0; c<m.cols; c++)
// 				{
// 					PP(r,c) = m.data[r*m.cols+c];
// 				}
// 			}
			QMat PP = innermodel->getTransformationMatrix("robot", "rgbd_t");
			
			RTMat object_tr( itMap->second.rx, itMap->second.ry, itMap->second.rz, itMap->second.tx, itMap->second.ty, itMap->second.tz );

							 
			const RTMat translated_obj = PP * object_tr;
			
			const QVec r = translated_obj.extractAnglesR_min();
			
// 			table->set_board_center(translated_obj(0,3), translated_obj(1,3), translated_obj(2,3));
// 			table->set_board_rotation(r(0), r(1), r(2));
// 			
// 			QVec table_translation = table->get_board_center();
// 			QVec table_rotation = table->get_board_rotation();
			
			r.print("april");
			
			RoboCompInnerModelManager::Pose3D pose;
			pose.x = translated_obj(0,3);
			pose.y = translated_obj(1,3);
			pose.z = translated_obj(2,3);
			pose.rx = r(0);
			pose.ry = r(1) + 0.1;
			pose.rz = r(2);
			
		// 			cout<<"Translation of the table: Tx: " <<pose.x<<" Ty: "<<pose.y<<" Tz: "<<pose.z<<endl;
		// 			cout<<"Rotation of table: Rx: "<<pose.rx<<" Ry: "<<pose.ry<<" Rz: "<<pose.rz<<endl;
			
			
			bool exists = false;
			
			RoboCompInnerModelManager::NodeInformationSequence node_sequence;
			innermodelmanager_proxy->getAllNodeInformation(node_sequence);
			for (unsigned int i=0; i<node_sequence.size(); i++)
			{
				if(node_sequence[i].id == "table")
				{
					exists = true;
					break;
				}
			}
			
			if(!exists)
				addTheTable(pose);
			else
				updateTheTable(pose);
	
			break;
 		}
	}
	drawTheTable();
	mutex->unlock();
	
}

void SpecificWorker::performEuclideanClustering()
{
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (this->cloud);
	cluster_indices.clear();
	cluster_clouds.clear();
	pcl::EuclideanClusterExtraction<PointT> ec;
	
	ec.setClusterTolerance (30); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
	
	ec.setInputCloud (this->cloud);
  ec.extract (cluster_indices);
	 
	int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
		{
      cloud_cluster->points.push_back (this->cloud->points[*pit]); //*
		}
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
		
		//save the cloud at 
		cluster_clouds.push_back(cloud_cluster);
		
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<PointT> (ss.str (), *cloud_cluster, false); //*
    j++;
  }
  euclidean_mutex->unlock();
}

void SpecificWorker::updatePointCloud()
{
	//transform to pcl cloud
	try
	{
		rgbd_proxy->getImage(rgbMatrix, distanceMatrix, points_kinect,  h, b);
	}
	catch(Ice::Exception e)
  {
		qDebug()<<"Error talking to rgbd_proxy: "<<e.what();
	}

	cloud->points.resize(points_kinect.size());

		
// 	RoboCompInnerModelManager::Matrix m = innermodelmanager_proxy->getTransformationMatrix("rgbd_t", "robot");
// 	QMat PP = QMat(m.rows, m.cols);
// 	for (int r=0; r<m.rows; r++)
// 	{
// 		for (int c=0; c<m.cols; c++)
// 		{
// 			PP(r,c) = m.data[r*m.cols+c];
// // 			std::cout<<PP(r,c)<<" ";
// 		}
// // 		std::cout<<std::endl;
// 	}
// // 	std::cout<<std::endl;
	
	QMat PP = innermodel->getTransformationMatrix("robot", "rgbd_t");
	
	
	bool first=true;
	for (unsigned int i=0; i<points_kinect.size(); i++)
	{
		QVec p1 = QVec::vec4(points_kinect[i].x, points_kinect[i].y, points_kinect[i].z, 1);
		QVec p2 = PP * p1;
		QVec p22 = p2.fromHomogeneousCoordinates();

		if (not isnan(points_kinect[i].x) and first)
		{
// 			p1.print("p1");
// 			p2.print("p2");
// 			p22.print("p22");
// 			first = false;
		}
			memcpy(&cloud->points[i],p22.data(),3*sizeof(float));
// 		cloud->points[i].x=p22(0);
// 		cloud->points[i].y=p22(1);
// 		cloud->points[i].z=p22(2);
		cloud->points[i].r=rgbMatrix[i].red;
		cloud->points[i].g=rgbMatrix[i].green;
		cloud->points[i].b=rgbMatrix[i].blue;
	}
	std::vector< int > index;
	removeNaNFromPointCloud (*cloud, *cloud, index);
	
	//lets make a copy to maintain the origianl cloud
	*original_cloud = *cloud;
	
	//Downsample the point cloud:
	
// 	pcl::VoxelGrid<PointT> sor;
//  sor.setInputCloud (cloud);
//  sor.setLeafSize (0.01f, 0.01f, 0.01f);
//  sor.filter (*downsampled_cloud);

// 	pcl::SampleConsensusModelPlane<PointT>::Ptr
//     model_p (new pcl::SampleConsensusModelPlane<PointT> (cloud));
// 	std::vector<int> inliers;
// 		 
// 	pcl::RandomSampleConsensus<PointT> ransac (model_p);
//   ransac.setDistanceThreshold (.01);
//   ransac.computeModel();
//   ransac.getInliers(inliers);
// 	std::cout<<"Inliers size: "<<inliers->size()<<std::endl;
// 	pcl::copyPointCloud<PointT>(*cloud, inliers, *cloud);
  
}

void SpecificWorker::drawThePointCloud(pcl::PointCloud<PointT>::Ptr cloud)
{
	//Now show results
	RoboCompInnerModelManager::PointCloudVector pointcloud;
	pointcloud.resize(cloud->size());
  int j=0;
  for (pcl::PointCloud<PointT>::iterator it = cloud->points.begin (); it < cloud->points.end (); ++it)
  {
		
		pointcloud[j].r=it->r;
		pointcloud[j].g=it->g;
		pointcloud[j].b=it->b;
		pointcloud[j].x=(it->x);
		pointcloud[j].y=(it->y);
		pointcloud[j].z=(it->z);

    j++;
  }
  
//   try
//   {
    add_point_cloud_to_innermodels("cloud", pointcloud);
		
//   }
//   catch(Ice::Exception e)
//   {
//     qDebug()<<"Error talking to inermodelmanager_proxy: "<<e.what();
//   } 
}

void SpecificWorker::doTheAprilTags()
{
	mutex->lock();
	for (TagModelMap::iterator itMap=tagMap.begin();  itMap!=tagMap.end(); itMap++)
	{
		//move the tags to the robot reference frame
		
		if (itMap->second.id == 3)
		{

			QMat PP = innermodel->getTransformationMatrix("robot", "rgbd_t");
			
			RTMat object_tr( itMap->second.rx, itMap->second.ry, itMap->second.rz, itMap->second.tx, itMap->second.ty, itMap->second.tz );

							 
			const RTMat translated_obj = PP * object_tr;
			
			const QVec tr = translated_obj.getTr();
			const QVec r = translated_obj.extractAnglesR_min();

			
			RoboCompInnerModelManager::Pose3D pose;
			pose.x = translated_obj(0,3);
			pose.y = translated_obj(1,3);
			pose.z = translated_obj(2,3);
			pose.rx = r(0);
			pose.ry = r(1);
			pose.rz = r(2);
			
			bool exists = false;
			
			RoboCompInnerModelManager::NodeInformationSequence node_sequence;
			innermodelmanager_proxy->getAllNodeInformation(node_sequence);
			for (unsigned int i=0; i<node_sequence.size(); i++)
			{
				if(node_sequence[i].id == "box")
				{
					exists = true;
					break;
				}
			}
			
			if(!exists)
			{
				std::cout<<"pintando"<<std::endl;
				addTheBox(pose);
			}
			else
				updateTheBox(pose);
			break;
 		}
	}
	mutex->unlock();
}

void SpecificWorker::addTheBox(RoboCompInnerModelManager::Pose3D pose)
{
	RoboCompInnerModelManager::Pose3D pose2;
	pose2.x = pose2.y = pose2.z = pose2.rx = pose2.ry = pose2.rz = 0;
	
	QVec box_size = box->get_size();

	RoboCompInnerModelManager::meshType box_mesh;
 	box_mesh.meshPath = "/home/robocomp/robocomp/files/osgModels/basics/cubexxx.3ds";
	box_mesh.pose.x = box_mesh.pose.y = box_mesh.pose.z = box_mesh.pose.rx = box_mesh.pose.ry = box_mesh.pose.rz = 0;
	box_mesh.scaleX = box_size(0);
	box_mesh.scaleY = box_size(1); 
	box_mesh.scaleZ = box_size(2); 
	box_mesh.render = 1; //set wireframe mode with 1 regular with 0
	
	// fixing the tag offset
	pose2.x = box_offset.x;
	pose2.z = box_offset.y;
	pose2.y = box_offset.z;    
	
	try
	{
		add_transform_to_innermodels("box_T",  "static", "robot", pose);
		add_transform_to_innermodels("box_T2",  "static", "box_T", pose2);
		add_mesh_to_innermodels("box", "box_T2", box_mesh);
		
		//add position to box
		RTMat box_transform = innermodel->getTransformationMatrix("robot", "box");
		QVec box_rotation = box_transform.extractAnglesR_min();
		
		box->set_center(QVec::vec3(box_transform(0,3), box_transform(1,3), box_transform(2,3)));
		box->set_rotation(QVec::vec3(box_rotation(0), box_rotation(1), box_rotation(2)));
		
	}
	catch(RoboCompInnerModelManager::InnerModelManagerError e)
	{
		std::cout<<e.text<<std::endl;
	}
	
}

void SpecificWorker::addTheTable(RoboCompInnerModelManager::Pose3D pose)
{
	RoboCompInnerModelManager::Pose3D pose2;
	pose2.x = pose2.y = pose2.z = pose2.rx = pose2.ry = pose2.rz = 0;
	
	QVec table_size = table->get_board_size();
	
	RoboCompInnerModelManager::meshType table_mesh;
 	table_mesh.meshPath = "/home/robocomp/robocomp/files/osgModels/basics/cubexxx.3ds";
	table_mesh.pose.x = table_mesh.pose.y = table_mesh.pose.z = table_mesh.pose.rx = table_mesh.pose.ry = table_mesh.pose.rz = 0;
	table_mesh.scaleX = table_size(0);
	table_mesh.scaleY = table_size(2);  //TAKE CARE IN THE MODEL Z AND Y ARE CHANGED!!!
	table_mesh.scaleZ = table_size(1);  //TAKE CARE IN THE MODEL Z AND Y ARE CHANGED!!! 
	table_mesh.render = 1; // set wireframe mode
	
	// fixing the tag offset

	//the axis o
	pose2.x = table_offset.x;
	pose2.y = table_offset.z;
	pose2.z = table_offset.y; 
	
	try
	{
		add_transform_to_innermodels("table_T",  "static", "robot", pose);
		add_transform_to_innermodels("table_T2",  "static", "table_T", pose2);
		add_mesh_to_innermodels("table", "table_T2", table_mesh);
		
		//add position to table
		RTMat table_transform = innermodel->getTransformationMatrix("robot", "table");
		QVec table_rotation = table_transform.extractAnglesR_min();
		
		table->set_board_center(table_transform(0,3), table_transform(1,3), table_transform(2,3));
		table->set_board_rotation(table_rotation(0), table_rotation(1), table_rotation(2));
		
	}
	catch(RoboCompInnerModelManager::InnerModelManagerError e)
	{
		std::cout<<e.text<<std::endl;
	}
	
}

void SpecificWorker::updateTheBox(RoboCompInnerModelManager::Pose3D pose)
{
	update_transforms_on_innermodels("box_T", pose);
	
	//add position to box
	RTMat box_transform = innermodel->getTransformationMatrix("robot", "box");
	QVec box_rotation = box_transform.extractAnglesR_min();
	
	box->set_center(QVec::vec3(box_transform(0,3), box_transform(1,3), box_transform(2,3)));
	box->set_rotation(QVec::vec3(box_rotation(0), box_rotation(1), box_rotation(2)));
}

void SpecificWorker::updateTheTable(RoboCompInnerModelManager::Pose3D pose)
{
 	update_transforms_on_innermodels("table_T", pose);
	
	//add position to table
	RTMat table_transform = innermodel->getTransformationMatrix("robot", "table");
	QVec table_rotation = table_transform.extractAnglesR_min();

	table->set_board_center(table_transform(0,3), table_transform(1,3), table_transform(2,3));
	table->set_board_rotation(table_rotation(0), table_rotation(1), table_rotation(2));
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
}

void SpecificWorker::newAprilTag(const tagsList& tags)
{
	mutex->lock();
	
	QTime c = QTime::currentTime();

	for (uint32_t i=0; i<tags.size(); i++)
	{
		if (tagMap.find(tags[i].id) != tagMap.end())
		{
			tagMap[tags[i].id] = AprilTagModel();
		}
		else
		{
			printf("new tag %d\n", tags[i].id);
		}

		tagMap[tags[i].id].id = tags[i].id;
		tagMap[tags[i].id].tx = tags[i].tx;
		tagMap[tags[i].id].ty = tags[i].ty;
		tagMap[tags[i].id].tz = tags[i].tz;
		tagMap[tags[i].id].rx = tags[i].rx;
		tagMap[tags[i].id].ry = tags[i].ry;
		tagMap[tags[i].id].rz = tags[i].rz;
		tagMap[tags[i].id].lastTime = c;
	}
	
	mutex->unlock();
}


bool SpecificWorker::add_point_cloud_to_innermodels(const std::string &id, const RoboCompInnerModelManager::PointCloudVector &cloud)
{
	//add to innermodel somehow //-- TODO
	
	//add to RCIS
	innermodelmanager_proxy->setPointCloudData(id, cloud);
}

bool SpecificWorker::add_transform_to_innermodels(const std::string &item, const std::string &engine, const std::string &base, const RoboCompInnerModelManager::Pose3D &pose)
{
	//adding to local innermodel
	InnerModelNode * parent = innermodel->getNode(QString::fromStdString(base));
	innermodel->newTransform(QString::fromStdString(item), QString::fromStdString("static") ,parent, pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz);
	cout<<"on local: "<<item<<" "<<pose.x<<" "<<pose.y<<" "<<pose.z<<endl;
	
	//adding to rcis:
	innermodelmanager_proxy->addTransform(item, engine, base, pose);
}

bool SpecificWorker::add_mesh_to_innermodels(const std::string &item, const std::string &base, const RoboCompInnerModelManager::meshType &m)
{
// 	//ading to local innermodel
	InnerModelNode * parent = innermodel->getNode(QString::fromStdString(base));
	innermodel->newMesh (
		QString::fromStdString(item),
		parent,
		QString::fromStdString(m.meshPath),
		m.scaleX, m.scaleY, m.scaleZ,
		m.render,
		m.pose.x, m.pose.y, m.pose.z,
		m.pose.rx, m.pose.ry, m.pose.rz);
	
	//adding to rcis	
	innermodelmanager_proxy->addMesh(item, base, m);
	
}

void SpecificWorker::update_transforms_on_innermodels (const std::string &item, const RoboCompInnerModelManager::Pose3D pose)
{
	//update on local innermodel
	innermodel->updateTransformValues(QString::fromStdString(item), pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz);
	//update on RCIS
	innermodelmanager_proxy->setPoseFromParent(item, pose);
}
