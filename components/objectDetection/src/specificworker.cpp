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

SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx), mutex(new QMutex()), point_cloud_mutex(new QMutex()), cloud(new pcl::PointCloud<PointT>), original_cloud(new pcl::PointCloud<PointT>),
segmented_cloud(new pcl::PointCloud<PointT>), model_inliers_indices(new pcl::PointIndices), plane_hull(new pcl::PointCloud<PointT>), cloud_hull(new pcl::PointCloud<PointT>)

{
	innermodel = new InnerModel("/home/robocomp/robocomp/components/perception/etc/genericPointCloud.xml");
	table_offset.x = -150;
	table_offset.y = 0;
	table_offset.z = 350;
	
	//action flags
	getTableInliers_flag = projectTableInliers_flag = tableConvexHull_flag = extractTablePolygon_flag = false;
	
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
		doTheBox();
	}
	if(model=="table")
	{
		doTheTable();
	}
}

void SpecificWorker::getInliers(const string& model)
{
	if(model=="table")
		getTableInliers_flag = !getTableInliers_flag;
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

void SpecificWorker::getTableInliers()
{
	std::cout<<"Getting Table Inliers"<<std::endl;
	
	pcl::PointCloud<PointT>::Ptr inliers_cloud(new pcl::PointCloud<PointT>);
	RoboCompInnerModelManager::Pose3D table_pose;
	RoboCompInnerModelManager::NodeInformation table_node;
	//get table model position
	RTMat transform = innermodel->getTransformationMatrix("robot", "table_T");

	PointT table_center;
	
	table_center.x = transform(0,3) + table_offset.x;
	table_center.y = transform(1,3) + table_offset.y;
	table_center.z = transform(2,3) + table_offset.z;
	
	std::cout<<" Table center: "<<std::endl;
	std::cout<<transform(0,3)<<std::endl;
	std::cout<<transform(1,3)<<std::endl;
	std::cout<<transform(2,3)<<std::endl;
	
 	model_inliers_indices->indices.resize(this->cloud->points.size());
	inliers_cloud->points.resize(this->cloud->points.size());

	int j=0;
	int index=0;
	for (pcl::PointCloud<PointT>::iterator it = cloud->points.begin (); it < cloud->points.end (); ++it)
  {
	
		if ((it->x) < table_center.x + tablesize.x && (it->x) > table_center.x - tablesize.x &&
				(it->y) < table_center.y + tablesize.y && (it->y) > table_center.y - 50 &&
				(it->z) < table_center.z + tablesize.z && (it->z) > table_center.z - tablesize.z)
		{
			inliers_cloud->points[j].x = (it->x);
			inliers_cloud->points[j].y = (it->y);
			inliers_cloud->points[j].z = (it->z);
			inliers_cloud->points[j].r = 0;
			inliers_cloud->points[j].g = 0;
			inliers_cloud->points[j].b = 255;
			
			model_inliers_indices->indices[j]=index;
			j++;
		}
		index++;;
  }
  
  inliers_cloud->points.resize(j);
  model_inliers_indices->indices.resize(j);
	
	*this->cloud = *inliers_cloud;
	
}

void SpecificWorker::projectTableInliers()
{
	RTMat transform = innermodel->getTransformationMatrix("robot", "table_T");
	
	//lets get three points of the plane 
	
	QVec p1 = transform * QVec::vec4(0, 0, 0, 1);
	QVec p2 = transform * QVec::vec4(1, 0, 0, 1);
	QVec p3 = transform * QVec::vec4(0, 1, 0, 1);
	
// 	p1.print("p1");
// 	p2.print("p2");
// 	p3.print("p3");
	
	
// 	QVec normal = (p2-p1) * (p3-p1);
// 	normal = normal.normalize();
// 	float d = normal * p1;
// 	
// 	cout<<" A: "<<normal(0)<<" B: "<<normal(1)<<" C: "<<normal(2)<<" D: "<<d<<endl;
	
	PointT p_1,p_2, p_3;
	p_1.x = p1(0);
	p_1.y = p1(1);
	p_1.z = p1(2);
	
	p_2.x = p2(0);
	p_2.y = p2(1);
	p_2.z = p2(2);
	
	p_3.x = p3(0);
	p_3.y = p3(1);
	p_3.z = p3(2);

	pcl::ModelCoefficients::Ptr plane (new pcl::ModelCoefficients);
	
	threePointsToPlane(p_1, p_2, p_3, plane);
	
// 	cout<<"PCL0: "<<plane->values[0]<<endl;
// 	cout<<"PCL1: "<<plane->values[1]<<endl;
// 	cout<<"PCL2: "<<plane->values[2]<<endl;
// 	cout<<"PCL3: "<<plane->values[3]<<endl;
	
// 	this->cloud=model_inliers_cloud;
  
  cout<<"Model inliers!!: "<<model_inliers_indices->indices.size()<<endl;

	//Let's project inliers
	pcl::ProjectInliers<PointT> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setIndices (model_inliers_indices);
	proj.setInputCloud (this->original_cloud);
	proj.setModelCoefficients (plane);
	proj.filter (*plane_hull);
	
	//update showing cloud
	*this->cloud = *plane_hull;
	
}

void SpecificWorker::tableConvexHull()
{
	//Let's construct a convex hull representation of the model inliers
	pcl::ConvexHull<PointT> chull;
	chull.setInputCloud(plane_hull);
	chull.reconstruct(*cloud_hull);
	
	*this->cloud = *cloud_hull;
}

//it only works for the table now
void SpecificWorker::extractPolygon(const string& model)
{
	if(model=="table")
	{
		extractTablePolygon_flag = ! extractTablePolygon_flag;
	}
}


void SpecificWorker::threePointsToPlane (const PointT &point_a, 
                            const PointT &point_b, 
                            const PointT &point_c, 
                            const pcl::ModelCoefficients::Ptr plane) 
{ 
  // Create Eigen plane through 3 points 
  Eigen::Hyperplane<float, 3> eigen_plane = 
    Eigen::Hyperplane<float, 3>::Through (point_a.getArray3fMap (), 
                                                          point_b.getArray3fMap (), 
                                                          point_c.getArray3fMap ()); 

  plane->values.resize (4); 

  for (int i = 0; i < plane->values.size (); i++) 
    plane->values[i] = eigen_plane.coeffs ()[i]; 
}

void SpecificWorker::extractTablePolygon()
{
	std::cout<<"remove point clouds within table"<<std::endl;
	
	//let's segment those points that are in the polinomial prism
	
	//obtain viewpoint 
	RTMat viewpoint_transform = innermodel->getTransformationMatrix("robot", "rgbd_t");
	
 	pcl::ExtractPolygonalPrismData<PointT> prism_extract;
 	pcl::PointIndices::Ptr prism_indices (new pcl::PointIndices);
 	
 	prism_extract.setHeightLimits(10, 1500);
	prism_extract.setViewPoint(0, 1500, 0);
 	prism_extract.setInputCloud(this->original_cloud);
 	prism_extract.setInputPlanarHull(cloud_hull);
 	prism_extract.segment(*prism_indices);
 	
	//let's extract the result
 	pcl::ExtractIndices<PointT> extract_prism_indices;
 	extract_prism_indices.setInputCloud(this->original_cloud);
 	extract_prism_indices.setIndices(prism_indices);
 	extract_prism_indices.filter(*(this->cloud));

}

void SpecificWorker::compute( )
{
		
		doThePointClouds();
		
		if(getTableInliers_flag)
			getTableInliers();
		
		if(projectTableInliers_flag)
			projectTableInliers();
		
		if(tableConvexHull_flag)
			tableConvexHull();
		
		if(extractTablePolygon_flag)
			extractTablePolygon();
		
		drawThePointCloud(this->cloud);
	
// 	doTheAprilTags();
	
}

void SpecificWorker::doTheBox()
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

void SpecificWorker::doTheTable()
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
				if(node_sequence[i].id == "table")
				{
					exists = true;
					break;
				}
			}
			
			if(!exists)
			{
				std::cout<<"pintando"<<std::endl;
				addTheTable(pose);
			}
			else
				updateTheTable(pose);
			break;
 		}
	}
	mutex->unlock();
	
}

void SpecificWorker::doThePointClouds()
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

		cloud->points[i].x=p22(0);
		cloud->points[i].y=p22(1);
		cloud->points[i].z=p22(2);
		cloud->points[i].r=rgbMatrix[i].red;
		cloud->points[i].g=rgbMatrix[i].green;
		cloud->points[i].b=rgbMatrix[i].blue;
	}
	
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

void SpecificWorker::addTheBox(RoboCompInnerModelManager::Pose3D pose)
{
	RoboCompInnerModelManager::Pose3D pose2;
	pose2.x = pose2.y = pose2.z = pose2.rx = pose2.ry = pose2.rz = 0;

	RoboCompInnerModelManager::meshType box_mesh;
 	box_mesh.meshPath = "/home/robocomp/robocomp/files/osgModels/basics/cubexxx.3ds";
	box_mesh.pose.x = box_mesh.pose.y = box_mesh.pose.z = box_mesh.pose.rx = box_mesh.pose.ry = box_mesh.pose.rz = 0;
	box_mesh.scaleX = 57.5;
	box_mesh.scaleY = 80; 
	box_mesh.scaleZ = 57.5; 
	box_mesh.render = 1; //set wireframe mode with 1 regular with 0
	
	// fixing the tag offset
	pose2.z = 65.5;
	pose2.y = 26;

	
	try
	{
		add_tranform_to_innermodels("box_T",  "static", "robot", pose);
		add_tranform_to_innermodels("box_T2",  "static", "box_T", pose2);
		add_mesh_to_innermodels("box", "box_T2", box_mesh);
// 		innermodelmanager_proxy->addTransform("box_T",  "static", "robot", pose);
// 		innermodelmanager_proxy->addTransform("box_T2", "static", "box_T", pose2);
// 		innermodelmanager_proxy->addMesh("box", "box_T2", box_mesh);
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
	
	RoboCompInnerModelManager::meshType table_mesh;
 	table_mesh.meshPath = "/home/robocomp/robocomp/files/osgModels/basics/cubexxx.3ds";
	table_mesh.pose.x = table_mesh.pose.y = table_mesh.pose.z = table_mesh.pose.rx = table_mesh.pose.ry = table_mesh.pose.rz = 0;
	table_mesh.scaleX = tablesize.x;
	table_mesh.scaleY = tablesize.z;  //TAKE CARE IN THE MODEL Z AND Y ARE CHANGED!!!
	table_mesh.scaleZ = tablesize.y;  //TAKE CARE IN THE MODEL Z AND Y ARE CHANGED!!! 
	table_mesh.render = 1; // set wireframe mode
	
	// fixing the tag offset

	//the axis o
	pose2.x = table_offset.x;
	pose2.y = table_offset.z;
	pose2.z = table_offset.y; 
	
	try
	{
		add_tranform_to_innermodels("table_T",  "static", "robot", pose);
		add_tranform_to_innermodels("table_T2",  "static", "table_T", pose2);
		add_mesh_to_innermodels("table", "table_T2", table_mesh);
		
	}
	catch(RoboCompInnerModelManager::InnerModelManagerError e)
	{
		std::cout<<e.text<<std::endl;
	}
	
}

void SpecificWorker::updateTheBox(RoboCompInnerModelManager::Pose3D pose)
{
	update_transforms_on_innermodels("box_T", pose);
}

void SpecificWorker::updateTheTable(RoboCompInnerModelManager::Pose3D pose)
{
 	update_transforms_on_innermodels("table_T", pose);
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
};
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

bool SpecificWorker::add_tranform_to_innermodels(const std::string &item, const std::string &engine, const std::string &base, const RoboCompInnerModelManager::Pose3D &pose)
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
