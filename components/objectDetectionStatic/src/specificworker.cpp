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
segmented_cloud(new pcl::PointCloud<PointT>), model_inliers_indices(new pcl::PointIndices), projected_plane(new pcl::PointCloud<PointT>), cloud_hull(new pcl::PointCloud<PointT>), 
euclidean_mutex(new QMutex()), cloud_to_normal_segment (new pcl::PointCloud<PointT>), prism_indices (new pcl::PointIndices), vfh_matcher(new VFH())

{
	innermodel = new InnerModel("/home/robocomp/robocomp/components/perception/etc/genericPointCloud.xml");
	
	viewpoint_transform = innermodel->getTransformationMatrix("robot", "rgbd_t");
	
	//let's set the sizes
	table->set_board_size(1000,30,300);
	box->set_size(QVec::vec3(57.5, 20.0, 57.5));

	//let's set the ofsets
	table_offset.x = -150;
	table_offset.y = 0;
	table_offset.z = 300;
	
	box_offset.x = 0;
	box_offset.y = 65.5;
	box_offset.z = 26;
	
	//action flags
	getTableInliers_flag = projectTableInliers_flag = tableConvexHull_flag = extractTablePolygon_flag = getTableRANSAC_flag = euclideanClustering_flag = objectSelected_flag = false;
	normal_segmentation_flag = false;
	
	saved_counter = 0;
	
	//only released when the euclidean cluster is performed
	euclidean_mutex->lock();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

void SpecificWorker::aprilFitModel(const string& model)
{
	if(model=="box")
	{
		aprilFitTheBox();
	}
	if(model=="table")
	{
		aprilFitTheTable();
	}
}

void SpecificWorker::fitModel(const string& model, const string& method)
{
	if(model=="prism")
    {
		if(method=="naive")
        {
			fitPrismtoObjectNaive();
        }
		else
        {
			if(method=="pf")
				fitPrismtoObjectPf();
        }
    }
}

void SpecificWorker::grabThePointCloud()
{
	updatePointCloud();
	drawThePointCloud(this->cloud);
}

void SpecificWorker::getInliers(const string& model)
{
	table->get_table_inliers(cloud, cloud, model_inliers_indices);
	drawThePointCloud(this->cloud);
}

void SpecificWorker::ransac(const string& model)
{
	
	QVec rotation = table->get_board_rotation();
// 		0	rotation.print("PREROTATION: ");
	table->fit_board_with_RANSAC( cloud, 0.01);
	drawTheTable();
	drawThePointCloud(this->cloud);
}

void SpecificWorker::projectInliers(const string& model)
{
	table->project_board_inliers(cloud, model_inliers_indices, projected_plane);

	//update showing cloud
	*this->cloud = *projected_plane;
	drawThePointCloud(this->cloud);
}

void SpecificWorker::convexHull(const string& model)
{
	table->board_convex_hull(projected_plane, cloud_hull);
	this->cloud = cloud_hull;
	drawThePointCloud(this->cloud);
}

//it only works for the table now
void SpecificWorker::extractPolygon(const string& model)
{
	table->extract_table_polygon(this->original_cloud, cloud_hull, QVec::vec3(viewpoint_transform(0,3), viewpoint_transform(1,3), viewpoint_transform(2,3)) , 20, 1500, prism_indices, this->cloud);
	drawThePointCloud(this->cloud);
	
}

void SpecificWorker::normalSegmentation(const string& model)
{
	table->normal_segmentation(this->original_cloud, 30, QVec::vec3(viewpoint_transform(0,3), viewpoint_transform(1,3), viewpoint_transform(2,3)), prism_indices, this->cloud);
	drawThePointCloud(this->cloud);
}

void SpecificWorker::euclideanClustering(int &num_clusters)
{
	//release flag to perferorm euclidean clustering
// 	euclideanClustering_flag = ! euclideanClustering_flag;
	
	
	performEuclideanClustering();
	//wait for euclidean clustering to be performed
	euclidean_mutex->lock();
	num_clusters = cluster_indices.size();
	std::cout<<"Euclidean clustring done: "<<num_clusters<<" clusters"<<std::endl;
}

void SpecificWorker::compute( )
{
		
// 		if(objectSelected_flag)
// 		{
// 			drawThePointCloud(cluster_clouds[object_to_show]);
// 		}
// 		else
// 			drawThePointCloud(this->cloud);
	
}
void SpecificWorker::showObject(int object_to_show)
{
	this->object_to_show = object_to_show;
	objectSelected_flag = true;
}

void SpecificWorker::mindTheGapPC()
{
	if(objectSelected_flag)
	{
		
		//-------------grab pcd
		
		pcl::PointCloud<PointT>::Ptr cl (new pcl::PointCloud<PointT>);
		
		QMat PP = innermodel->getTransformationMatrix("rgbd_t", "robot");
		
		cl->points.resize(cluster_clouds[object_to_show]->points.size());

		for (unsigned int i=0; i<cluster_clouds[object_to_show]->points.size(); i++)
		{
			QVec p1 = QVec::vec4(cluster_clouds[object_to_show]->points[i].x, cluster_clouds[object_to_show]->points[i].y, cluster_clouds[object_to_show]->points[i].z, 1);
			QVec p2 = PP * p1;
			QVec p22 = p2.fromHomogeneousCoordinates();

// 	// 			memcpy(&cloud->points[i],p22.data(),3*sizeof(float));
			cl->points[i].x=p22(0);
			cl->points[i].y=p22(1);
			cl->points[i].z=p22(2);
			cl->points[i].r=cluster_clouds[object_to_show]->points[i].r;
			cl->points[i].g=cluster_clouds[object_to_show]->points[i].g;
			cl->points[i].b=cluster_clouds[object_to_show]->points[i].b;
		}
// 	std::vector< int > index;
// 	removeNaNFromPointCloud (*cloud, *cloud, index);
// 	
// 	//lets make a copy to maintain the origianl cloud
// 	*original_cloud = *cloud;
// 	
		cl->width = 1;
		cl->height = cluster_clouds[object_to_show]->points.size();
		writer.write<PointT> ("box.pcd", *cl, false);
		
		//------------end of pcd grab
		
		
		Mirror mirror;
		mirror.setFittingParams();
		mirror.setDeviceParams();
		
		QVec plane_coeff = table->get_plane_coeff();
		plane_coeff.print("plane_coeff");
		
		std::vector<double> plane_coeff_std;
		plane_coeff_std.resize(4);
		plane_coeff_std[0] = plane_coeff(0);
		plane_coeff_std[1] = plane_coeff(1);
		plane_coeff_std[2] = plane_coeff(2);
		plane_coeff_std[3] = plane_coeff(3);
		mirror.setTablePlane( plane_coeff_std );
		
		int candidate = mirror.complete(cluster_clouds[object_to_show]);
		
		std::cout<<"Completed best candidate: "<<candidate<<std::endl;
		
// 		for(int i = 0; i < mindgapper.getNumCandidates(); i ++)
// 			mindgapper.viewMirror(i);
		
 	  writer.write<PointT> ("completed.pcd", *cluster_clouds[object_to_show], false); //*
	}
	else
		std::cout<<"Please select an object first"<<std::endl;
}

void SpecificWorker::centroidBasedPose()
{
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid (*(this->cloud), centroid);
	
	std::cout<<"Centroid : "<<centroid(0)<<" "<<centroid(1)<<" "<<centroid(2)<<std::endl;
	
	
}

void SpecificWorker::mirrorPC()
{
	
	Mirror mirror;
	QVec plane_coeff = table->get_plane_coeff();
	
	std::vector<double> plane_coeff_std;
	plane_coeff_std.resize(4);
	plane_coeff_std[0] = plane_coeff(0);
	plane_coeff_std[1] = plane_coeff(1);
	plane_coeff_std[2] = plane_coeff(2);
	plane_coeff_std[3] = plane_coeff(3);
	mirror.setTablePlane( plane_coeff_std );
	
	//moving the object point cloud to the rgbd
	pcl::PointCloud<PointT>::Ptr cl (new pcl::PointCloud<PointT>);
		
	QMat PP = innermodel->getTransformationMatrix("rgbd_t", "robot");
	
	cl->points.resize(cluster_clouds[object_to_show]->points.size());
	cl->width = 1;
	cl->height = cluster_clouds[object_to_show]->points.size();
	
	for (unsigned int i=0; i<cluster_clouds[object_to_show]->points.size(); i++)
	{
		QVec p1 = QVec::vec4(cluster_clouds[object_to_show]->points[i].x, cluster_clouds[object_to_show]->points[i].y, cluster_clouds[object_to_show]->points[i].z, 1);
		QVec p2 = PP * p1;
		QVec p22 = p2.fromHomogeneousCoordinates();

		cl->points[i].x=p22(0);
		cl->points[i].y=p22(1);
		cl->points[i].z=p22(2);
		cl->points[i].r=cluster_clouds[object_to_show]->points[i].r;
		cl->points[i].g=cluster_clouds[object_to_show]->points[i].g;
		cl->points[i].b=cluster_clouds[object_to_show]->points[i].b;
	}
	
	writer.write<PointT> ("box_test.pcd", *cl, false);

	
	mirror.centroidBasedComplete(cl);
	
	//leaving the object respect to the robot
	PP = innermodel->getTransformationMatrix("robot", "rgbd_t");
// 	cluster_clouds[object_to_show]->clear();
// 	cluster_clouds[object_to_show]->points.resize(cl->points.size());
// 	cluster_clouds[object_to_show]->width = 1;
// 	cluster_clouds[object_to_show]->height = cl->points.size();
	
	for (unsigned int i=0; i<cl->points.size(); i++)
	{
		QVec p1 = QVec::vec4(cl->points[i].x, cl->points[i].y, cl->points[i].z, 1);
		QVec p2 = PP * p1;
		QVec p22 = p2.fromHomogeneousCoordinates();

		cl->points[i].x=p22(0);
		cl->points[i].y=p22(1);
		cl->points[i].z=p22(2);
		cl->points[i].r=cluster_clouds[object_to_show]->points[i].r;
		cl->points[i].g=cluster_clouds[object_to_show]->points[i].g;
		cl->points[i].b=cluster_clouds[object_to_show]->points[i].b;
	}
	
	
	//3.- Reason for the what I should see point cloud
	cv::Mat M(480,640,CV_8UC1, cv::Scalar::all(0));
	for (int i = 0; i<cluster_clouds[object_to_show]->points.size(); i++)
	{
		QVec xy = innermodel->project("robot", QVec::vec3(cluster_clouds[object_to_show]->points[i].x, cluster_clouds[object_to_show]->points[i].y, cluster_clouds[object_to_show]->points[i].z), "rgbd"); 

		if (xy(0)>=0 and xy(0) < 640 and xy(1)>=0 and xy(1) < 480 )
		{
			M.at<uchar> ((int)xy(1), (int)xy(0)) = 255;
		}
		else if (not (isinf(xy(1)) or isinf(xy(0))))
		{
			std::cout<<"Accediendo a -noinf: "<<xy(1)<<" "<<xy(0)<<std::endl;
		}
	}
	
	//dilate
	cv::Mat dilated_M, z;
	cv::dilate( M, dilated_M, cv::Mat(), cv::Point(-1, -1), 2, 1, 1 );
	
	//find contour
	vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;
	
	cv::findContours( dilated_M, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
	
		/// Draw contours
	cv::Mat mask = cv::Mat::zeros( dilated_M.size(), CV_8UC1 );
// 		int contour_index = 1;

// 		cv::Scalar color = cv::Scalar( 0, 255, 0 );
// 		cv::drawContours( drawing, contours, contour_index, color, 2, 8, hierarchy, 0, cv::Point() );
	
	cv::drawContours(mask, contours, -1, cv::Scalar(255, 255, 255), CV_FILLED);
	
// 	for (int x = 0; x < 480; x++)
// 	{
// 		for (int y = 0; y < 640; y++)
// 		{
// 			
// 			std::cout<<(int)mask.at<uchar> (x, y);
// 		}
// 	}
	
	//check if points lie in the mask
	for (int i = 0; i<cl->points.size(); i++)
	{
		QVec xy = innermodel->project("robot", QVec::vec3(cl->points[i].x, cl->points[i].y, cl->points[i].z), "rgbd"); 
		if (xy(0)>=0 and xy(0) < 640 and xy(1)>=0 and xy(1) < 480 )
		{
			if ((int)mask.at<uchar> (xy(1), xy(0)) == 255 )
			{
				std::cout<<"Inside"<<std::endl;
				cluster_clouds[object_to_show]->points.push_back(cl->points[i]);
			}
		}
		else if (not (isinf(xy(1)) or isinf(xy(0))))
		{
			std::cout<<"Accediendo a -noinf: "<<xy(1)<<" "<<xy(0)<<std::endl;
		}
	}

	
	cv::namedWindow( "Display what I see", cv::WINDOW_AUTOSIZE );// Create a window for display.
  cv::imshow( "Display what I see", mask );
	
	//lets put them together
// 	*cluster_clouds[object_to_show] += *cl;
	
	std::cout<<"Mirror ended"<<std::endl;
	
}

void SpecificWorker::passThrough()
{
	// Create the filtering object
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (this->cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1100);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*this->cloud);
  drawThePointCloud(this->cloud);
}

void SpecificWorker::surfHomography(std::vector<string> &guesses)
{
	
}

void SpecificWorker::fitPrismtoObjectPf()
{
	if(objectSelected_flag)
	{
		//using the synthethic instead:
// 		pcl::PointCloud<PointT>::Ptr thecloud = generate_sinthetic_cube();
// 		std::cout<<thecloud->points.size()<<std::cout;
// 		drawThePointCloud(thecloud);
		//init fitter
		boost::shared_ptr<RectPrism> shape(new RectPrism());
		
		pf_fitter = new PfRectPrismFitting( 200, cluster_clouds[object_to_show] );
		 
// 		boost::shared_ptr<RectPrism> shape(new RectPrism());
//      pf_fitter = new naiveRectangularPrismFitting( thecloud );
 		
 		boost::function<void (const boost::shared_ptr<RectPrism>&)> f = boost::bind (&SpecificWorker::naive_fit_cb, this, _1);
		
		//Insert and draw cube intialization
		std::cout<<"Drawing the awesome cube"<<std::endl;
		
		const boost::shared_ptr<RectPrism> prism_fit = pf_fitter->getBest();
		
		QVec translation = prism_fit->get_center();
		QVec rotation = prism_fit->get_rotation();
		QVec size = prism_fit->get_size();
		
		RoboCompInnerModelManager::meshType prism_mesh;
		prism_mesh.meshPath = "/home/robocomp/robocomp/files/osgModels/basics/cubexxx.3ds";
		prism_mesh.pose.x = prism_mesh.pose.y = prism_mesh.pose.z = prism_mesh.pose.rx = prism_mesh.pose.ry = prism_mesh.pose.rz = 0;
		prism_mesh.scaleX = size(0)/2;
		prism_mesh.scaleY = size(1)/2; 
		prism_mesh.scaleZ = size(2)/2;
		prism_mesh.render = 1;
		
		RoboCompInnerModelManager::Pose3D pose;
		pose.x = translation(0);
		pose.y = translation(1);
		pose.z = translation(2);
		pose.rx = rotation(0);
		pose.ry = rotation(1);
		pose.rz = rotation(2);
		
		
		add_transform_to_innermodels("prism_t",  "static", "robot", pose);
		add_mesh_to_innermodels("prism", "prism_t", prism_mesh);
		
		
		pf_fitter->registerCallback (f);
		
		pf_fitter->start ();
	}
}

void SpecificWorker::fitPrismtoObjectNaive()
{
	if(objectSelected_flag)
	{
		//using the synthethic instead:
// 		pcl::PointCloud<PointT>::Ptr thecloud = generate_sinthetic_cube();
// 		std::cout<<thecloud->points.size()<<std::endl;
// 		drawThePointCloud(thecloud);
		//init fitter
		boost::shared_ptr<RectPrism> shape(new RectPrism());
		
		naive_fitter = new naiveRectangularPrismFitting( cluster_clouds[object_to_show] );
		 
// 		boost::shared_ptr<RectPrism> shape(new RectPrism());
//      naive_fitter = new naiveRectangularPrismFitting( thecloud );
 		
 		boost::function<void (const boost::shared_ptr<RectPrism>&)> f = boost::bind (&SpecificWorker::naive_fit_cb, this, _1);
		
		//Insert and draw cube intialization
		std::cout<<"Drawing the awesome cube"<<std::endl;
		
		const boost::shared_ptr<RectPrism> prism_fit = naive_fitter->getBest();
		
		QVec translation = prism_fit->get_center();
		QVec rotation = prism_fit->get_rotation();
		QVec size = prism_fit->get_size();
		
		RoboCompInnerModelManager::meshType prism_mesh;
		prism_mesh.meshPath = "/home/robocomp/robocomp/files/osgModels/basics/cubexxx.3ds";
		prism_mesh.pose.x = prism_mesh.pose.y = prism_mesh.pose.z = prism_mesh.pose.rx = prism_mesh.pose.ry = prism_mesh.pose.rz = 0;
		prism_mesh.scaleX = size(0)/2;
		prism_mesh.scaleY = size(1)/2; 
		prism_mesh.scaleZ = size(2)/2;
		prism_mesh.render = 1;
		
		RoboCompInnerModelManager::Pose3D pose;
		pose.x = translation(0);
		pose.y = translation(1);
		pose.z = translation(2);
		pose.rx = rotation(0);
		pose.ry = rotation(1);
		pose.rz = rotation(2);
		
		
		add_transform_to_innermodels("prism_t",  "static", "robot", pose);
		add_mesh_to_innermodels("prism", "prism_t", prism_mesh);
		
		
		naive_fitter->registerCallback (f);
		
		naive_fitter->start ();
	}
}

pcl::PointCloud<PointT>::Ptr SpecificWorker::generate_sinthetic_cube(const int tx, const int ty, const int tz, const int Wx, const int Wy, const int Wz, const int res)
{
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  //Rot3D r(0.5, 0.2, 0.2);
  //Faces front and back
  for(float x=tx; x<=Wx; x=x+res)
  {
    for(float y=ty; y<=Wx; y=y+res)
    {
      //face front (x=0)
      pcl::PointXYZRGB p;
      p.x = x;
      p.y = y;
      p.z = tz;
      cloud->push_back(p);
      p.x = x;
      p.y = y;
      p.z = Wz;      
      cloud->push_back(p);
    }
  }
  //Faces up and down
  for(float x=tx; x<=Wx; x=x+res)
  {
    for(float z=tz; z<=Wz; z=z+res)
    {
      //face front (x=0)
      pcl::PointXYZRGB p;
      p.x = x;
      p.y = ty;
      p.z = z;
      cloud->push_back(p);
      p.x = x;
      p.y = Wy;
      p.z = z;      
      cloud->push_back(p);
    }
  }
  //Faces right and left
  for(float y=ty; y<=Wy; y=y+res)
  {
    for(float z=tz; z<=Wz; z=z+res)
    {
      //face front (x=0)
      pcl::PointXYZRGB p;
      p.x = tx;
      p.y = y;
      p.z = z;
      cloud->push_back(p);
      p.x = Wx;
      p.y = y;
      p.z = z;      
      cloud->push_back(p);
    }
  }
  
  return cloud;
}

void SpecificWorker::naive_fit_cb (const boost::shared_ptr<RectPrism>  &shape)
{
	//update rectprism
  static int tick = 0;

  printf("\n------ Tick %d\n", tick++);
  QVec rotation = shape->get_rotation();
  
  
  //FIRST ROTATION: X
  RTMat rx = RTMat(rotation(0),0, 0, QVec::vec3(0,0,0));
  
  rx.print("Rx: ");
  
  //SECOND ROTATION: Y
  QVec y_prima = rx.getR().transpose()*QVec::vec3(0,1,0);
  
  y_prima.print("y_prima: ");
    
  Eigen::Vector3f k_vector1(y_prima(0),y_prima(1),y_prima(2));
  Eigen::Affine3f rotate1 = (Eigen::Affine3f) Eigen::AngleAxisf(rotation(1), k_vector1);
  
  RTMat rotate1_rtmat;
  rotate1_rtmat(0,0)=rotate1(0,0);  rotate1_rtmat(0,1)=rotate1(0,1);  rotate1_rtmat(0,2)= rotate1(0,2); rotate1_rtmat(0,3)= rotate1(0,3);
  rotate1_rtmat(1,0)=rotate1(1,0);  rotate1_rtmat(1,1)=rotate1(1,1);  rotate1_rtmat(1,2)= rotate1(1,2); rotate1_rtmat(1,3)= rotate1(1,3);
  rotate1_rtmat(2,0)=rotate1(2,0);  rotate1_rtmat(2,1)=rotate1(2,1);  rotate1_rtmat(2,2)= rotate1(2,2); rotate1_rtmat(2,3)= rotate1(2,3);
  rotate1_rtmat(3,0)=rotate1(3,0);  rotate1_rtmat(3,1)=rotate1(3,1);  rotate1_rtmat(3,2)= rotate1(3,2); rotate1_rtmat(3,3)= rotate1(3,3);
  
  RTMat rx_y = rx*rotate1_rtmat;
  
  rx_y.print("rx_y_mat: ");
  
  rx_y.extractAnglesR().print("rx_y");
  
  //THIRD ROTATION: Z
  QVec z_prima = rx_y.getR().transpose()*QVec::vec3(0,0,1);
  
  z_prima.print("z_prima: ");
  
  Eigen::Vector3f k_vector2(z_prima(0),z_prima(1),z_prima(2));
  Eigen::Affine3f rotate2 = (Eigen::Affine3f) Eigen::AngleAxisf(rotation(2), k_vector2);
  
  RTMat rotate2_rtmat;
  rotate2_rtmat(0,0)=rotate2(0,0);  rotate2_rtmat(0,1)=rotate2(0,1);  rotate2_rtmat(0,2)= rotate2(0,2); rotate2_rtmat(0,3)= rotate2(0,3);
  rotate2_rtmat(1,0)=rotate2(1,0);  rotate2_rtmat(1,1)=rotate2(1,1);  rotate2_rtmat(1,2)= rotate2(1,2); rotate2_rtmat(1,3)= rotate2(1,3);
  rotate2_rtmat(2,0)=rotate2(2,0);  rotate2_rtmat(2,1)=rotate2(2,1);  rotate2_rtmat(2,2)= rotate2(2,2); rotate2_rtmat(2,3)= rotate2(2,3);
  rotate2_rtmat(3,0)=rotate2(3,0);  rotate2_rtmat(3,1)=rotate2(3,1);  rotate2_rtmat(3,2)= rotate2(3,2); rotate2_rtmat(3,3)= rotate2(3,3);
  
  RTMat rotationresult = rx_y*rotate2_rtmat;
  rotationresult.print("rotationresult: ");
    
  QVec anglesresult = rotationresult.extractAnglesR();
  
  
  QVec center = shape->get_center();
	
	QVec size = shape->get_size();
	
	
	RoboCompInnerModelManager::Pose3D pose;
	pose.x = center(0);
			pose.y = center(1);
			pose.z = center(2);
			pose.rx = anglesresult(0);
			pose.ry = anglesresult(1);
			pose.rz = anglesresult(2);
	
// 	std::cout<<"Best weight: "<<naive_fitter->getBestWeight()<<std::endl;
			
	update_transforms_on_innermodels("prism_t", pose);
	innermodelmanager_proxy->setScale("prism", size(0)/2, size(1)/2, size(2)/2);
	
// 	v->setPose("cube_0_t", shape->getCenter(), shape->getRotation(), shape->getWidth() );
// 	v->setScale("cube_0", shape->getWidth()(0)/2, shape->getWidth()(1)/2, shape->getWidth()(2)/2);
// 	
// 	v->setPose("cube_best_t", fitter->getBest()->getCenter(), fitter->getBest()->getRotation(), fitter->getBest()->getWidth() );
// 	v->setScale("cube_best", fitter->getBest()->getWidth()(0)/2, fitter->getBest()->getWidth()(1)/2, fitter->getBest()->getWidth()(2)/2);
}

void SpecificWorker::reloadVFH()
{
// 	vfh_matcher->reloadVFH("/home/robocomp/robocomp/files/objectData/Rockin_set_labeled/");
	vfh_matcher->reloadVFH("/home/spyke/robocomp/components/perception/components/objectDetectionStatic/build/test/");
}

void SpecificWorker::loadTrainedVFH()
{
	vfh_matcher->loadTrainingData();
	std::cout<<"Training data loaded"<<std::endl;
}

void SpecificWorker::vfh(std::vector<string> &guesses)
{
	if(objectSelected_flag)
	{
		std::cout<<"[DEBUG] Cluster SIZE: "<<cluster_clouds[object_to_show]->points.size()<<std::endl;
		vfh_matcher->doTheGuess(cluster_clouds[object_to_show], vfh_guesses);
		guesses = vfh_guesses;
	}
	else
	{
		vfh_matcher->doTheGuess(this->cloud, vfh_guesses);
		guesses = vfh_guesses;
	}
}

void SpecificWorker::fitTheViewVFH()
{
	static bool first3 = true;
	static bool first0 = true;
	
	pcl::PointCloud<PointT>::Ptr cloud_to_fit (new pcl::PointCloud<PointT>);
	if(objectSelected_flag)
	{
		cloud_to_fit = cluster_clouds[object_to_show];
	}
	else
	{
		InnerModelNode *parent = innermodel->getNode(QString::fromStdString("rgbd"));
		innermodel->newTransform(QString::fromStdString("marca"), QString::fromStdString("static") ,parent, 356.152, -289.623, 756.853, 0.32, 0, 0);
		QMat PP = innermodel->getTransformationMatrix("marca", "robot");

		for (unsigned int i=0; i<this->cloud->points.size(); i++)
		{
			QVec p1 = QVec::vec4(this->cloud->points[i].x,this->cloud->points[i].y, this->cloud->points[i].z, 1);
			QVec p2 = PP * p1;
			QVec p22 = p2.fromHomogeneousCoordinates();
			
			PointT p;

			p.x=p22(0);
			p.y=p22(1);
			p.z=p22(2);
			p.r=this->cloud->points[i].r;
			p.g=this->cloud->points[i].g;
			p.b=this->cloud->points[i].b;
			cloud_to_fit->push_back(p);
		}
		writer.write<PointT> ("tofit.pcd", *cloud_to_fit, false); 
		//cloud_to_fit = this->cloud;
	}
	
	if(!vfh_guesses.empty())
	{
		
		FeatureCloudT::Ptr  object_features(new pcl::PointCloud<pcl::FPFHSignature33>);
		FeatureCloudT::Ptr  scene_features(new pcl::PointCloud<pcl::FPFHSignature33>);
		pcl::PointCloud<PointT>::Ptr object (new pcl::PointCloud<PointT>);
// 		pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);
		PointCloudT::Ptr scene_normals (new PointCloudT);
		PointCloudT::Ptr object_normals (new PointCloudT);
		pcl::PointCloud<PointT>::Ptr object_aligned (new pcl::PointCloud<PointT>);
		
		//read object view
		string filename = vfh_guesses[0].substr(0,vfh_guesses[0].find_last_of(".")) + ".pcd";
		
		if (pcl::io::loadPCDFile<PointT> (filename, *object) < 0)
		{
			pcl::console::print_error ("Error loading object/scene file: ", filename.c_str());
			return;
		}
		
		//Estimate normals
		pcl::console::print_highlight ("Estimating scene normals...\n");
		pcl::NormalEstimationOMP<PointT,pcl::PointNormal> nest;
		nest.setRadiusSearch (10);
		nest.setInputCloud (cloud_to_fit);
		nest.compute (*scene_normals);
		
		nest.setInputCloud (object);
		nest.compute (*object_normals);
		
		// Estimate features
		pcl::console::print_highlight ("Estimating features...\n");
		FeatureEstimationT fest;
		fest.setRadiusSearch (25);
		fest.setInputCloud (object);
		fest.setInputNormals (object_normals);
		fest.compute (*object_features);
		fest.setInputCloud (cloud_to_fit);
		fest.setInputNormals (scene_normals);
		fest.compute (*scene_features);
		
		// Perform alignment
		pcl::console::print_highlight ("Starting alignment...\n");
		pcl::SampleConsensusPrerejective<PointT,PointT,FeatureT> align;
		align.setInputSource (object);
		align.setSourceFeatures (object_features);
		align.setInputTarget (cloud_to_fit);
		align.setTargetFeatures (scene_features);
		align.setMaximumIterations (10000); // Number of RANSAC iterations
		align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
		align.setCorrespondenceRandomness (2); // Number of nearest features to use
		align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
		align.setMaxCorrespondenceDistance (1500); // Inlier threshold
		align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
		{
			pcl::ScopeTime t("Alignment");
			align.align (*object_aligned);
		}
		
		
		if (align.hasConverged ())
		{
			Eigen::Matrix4f transformation = align.getFinalTransformation ();
			
			//calculate transform and rotation
			
			//get the filename with no extension, it is a number corresponding to its rotation
			string filename_no_extension = vfh_guesses[0].substr(vfh_guesses[0].find_last_of("/")+1);
			filename_no_extension = filename_no_extension.substr(0,filename_no_extension.find_last_of("."));
			//dataset of views are rotated 24 deg., 0.41887902 rad
			int rotation_numb = atoi( filename_no_extension.c_str() ) - 1;
// 			float rotation = 0.41887902*rotation_numb;
			float rotation =0;

			
			InnerModel innermodel_location;
// 			InnerModelNode *parent = innermodel_location.getNode(QString::fromStdString("root"));
// 			innermodel_location.newTransform(QString::fromStdString("marca_original"), QString::fromStdString("static") ,parent, 356.152, -289.623, 756.853, 0.32, 0.0, 0);

			InnerModelNode *parent = innermodel_location.getNode(QString::fromStdString("root"));
			innermodel_location.newTransform(QString::fromStdString("vista_original"), QString::fromStdString("static") ,parent, -380, 290, 0, 0, rotation, 0);
			
			RTMat trans_ =  innermodel_location.getTransformationMatrix("root","vista_original");
			std::cout<<"trans de root a vista_original:"<<std::endl;
			std::cout<<"Tx: "<<trans_(0,3)<<" Ty: "<<trans_(1,3)<<" Tz: "<<trans_(2,3)<<std::endl;
			
			RTMat transform_view_to_object;
			
			transform_view_to_object(0,0) = transformation (0,0);
			transform_view_to_object(1,0) = transformation (1,0);
			transform_view_to_object(2,0) = transformation (2,0);
			
			transform_view_to_object(0,1) = transformation (0,1);
			transform_view_to_object(1,1) = transformation (1,1);
			transform_view_to_object(2,1) = transformation (2,1);
			
			transform_view_to_object(0,2) = transformation (0,2);
			transform_view_to_object(1,2) = transformation (1,2);
			transform_view_to_object(2,2) = transformation (2,2);
			
			transform_view_to_object(0,3) = transformation (0,3);
			transform_view_to_object(1,3) = transformation (1,3);
			transform_view_to_object(2,3) = transformation (2,3);
			
// 			transform_view_to_object.invert();
			
			
			QVec tr = QVec::vec3(transform_view_to_object(0,3),transform_view_to_object(1,3), transform_view_to_object(2,3));
			QVec r = transform_view_to_object.extractAnglesR_min();
			
			std::cout<<"Trans de vista_original a real"<<std::endl;
			std::cout<<"Tx: "<<tr(0)<<" Ty: "<<tr(1)<<" Tz: "<<tr(2)<<std::endl;
			std::cout<<"Rx: "<<r(0)<<" Ry: "<<r(1)<<" Rz: "<<r(2)<<std::endl;
			
			parent = innermodel_location.getNode(QString::fromStdString("vista_original"));
			innermodel_location.newTransform(QString::fromStdString("vista_real"), QString::fromStdString("static") ,parent, tr(0), tr(1), tr(2), r(0), r(1), r(2));
			
// 			parent = innermodel_location.getNode(QString::fromStdString("root"));
// 			innermodel_location.newTransform(QString::fromStdString("marca_real"), QString::fromStdString("static") ,parent, 356.152, -289.623, 756.853, 0.32, 0.0657813, -0.0324503);
			
// 			trans_ =  innermodel_location.getTransformationMatrix("root","vista_real");
// 			std::cout<<"trans de root a vista_Real:"<<std::endl;
// 			std::cout<<"Tx: "<<trans_(0,3)<<" Ty: "<<trans_(1,3)<<" Tz: "<<trans_(2,3)<<std::endl;
			
			RTMat t_wrt_table;
			t_wrt_table = innermodel_location.getTransformationMatrix("root","vista_real");
			
			tr = QVec::vec3(t_wrt_table(0,3), t_wrt_table(1,3), t_wrt_table(2,3));
			r = t_wrt_table.extractAnglesR_min();
			std::cout<<"trans de vista_Real a la marca (root):"<<std::endl;
			std::cout<<"Tx: "<<tr(0)<<" Ty: "<<tr(1)<<" Tz: "<<tr(2)<<std::endl;
			std::cout<<"Rx: "<<r(0)<<" Ry: "<<r(1)<<" Rz: "<<r(2)<<std::endl;
			
// 			innermodel_location->newTransform(QString::fromStdString(""), QString::fromStdString("static") ,parent, 380, 290, 0, 0, 0,0);
			
			// Print results
			printf ("\n");
			pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
			pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
			pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
			pcl::console::print_info ("\n");
			pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
			pcl::console::print_info ("\n");
			pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
			
			// Show alignment
			pcl::visualization::PCLVisualizer visu("Alignment");
			visu.addPointCloud (cloud_to_fit, ColorHandlerT (cloud_to_fit, 0.0, 255.0, 0.0), "scene");
			visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
			visu.spin ();
		}
		else
		{
			pcl::console::print_error ("Alignment failed!\n");
			return;
		}
		
	}
	//to test rotation and translation from one mark to another
	else
	{
		InnerModel innermodel_pos;
		mutex->lock();
		
		for (TagModelMap::iterator itMap=tagMap.begin();  itMap!=tagMap.end(); itMap++)
		{
			if (itMap->second.id == 3)
			{
				if(first3)
				{
					InnerModelNode *parent = innermodel->getNode(QString::fromStdString("root"));
					innermodel->newTransform(QString::fromStdString("tag3"), QString::fromStdString("static") ,parent, itMap->second.tx, itMap->second.ty, itMap->second.tz, itMap->second.rx, itMap->second.ry, itMap->second.rz);
					first3 = false;
				}
				else
				{
					innermodel->updateTransformValues("tag3", itMap->second.tx, itMap->second.ty, itMap->second.tz, itMap->second.rx, itMap->second.ry, itMap->second.rz, "root");
				}
			}
			if (itMap->second.id == 0)
			{
				if(first0)
				{
					InnerModelNode *parent = innermodel->getNode(QString::fromStdString("root"));
					innermodel->newTransform(QString::fromStdString("tag0"), QString::fromStdString("static") ,parent, itMap->second.tx, itMap->second.ty, itMap->second.tz, itMap->second.rx, itMap->second.ry, itMap->second.rz);
					first0 = false;
				}
				else
				{
					innermodel->updateTransformValues("tag0", itMap->second.tx, itMap->second.ty, itMap->second.tz, itMap->second.rx, itMap->second.ry, itMap->second.rz, "root");
				}
			}
		}
		mutex->unlock();
		
// 		const RTMat transform = innermodel->getTransformationMatrix("tag0", "tag3");
// 		const QVec tr = transform.getTr();
// 		const QVec r = transform.extractAnglesR_min();
// 		
// 		std::cout<<"Tx: "<<tr(0)<<" Ty: "<<tr(1)<<" Tz: "<<tr(2)<<std::endl;
// 		std::cout<<"Rx: "<<r(0)<<" Ry: "<<r(1)<<" Rz: "<<r(2)<<std::endl;
	}
		
}

void SpecificWorker::reset()
{
	//action flags
	getTableInliers_flag = projectTableInliers_flag = tableConvexHull_flag = extractTablePolygon_flag = getTableRANSAC_flag = euclideanClustering_flag = objectSelected_flag = false;
}

void SpecificWorker::aprilFitTheBox()
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

void SpecificWorker::aprilFitTheTable()
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
			pose.ry = r(1); // + 0.1; da noise!
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

void SpecificWorker::statisticalOutliersRemoval()
{
	// Create the filtering object
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud (this->cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*(this->cloud));
}

void SpecificWorker::performEuclideanClustering()
{
// 	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> ());
	
// 	//downsample
// 	pcl::VoxelGrid<PointT> sor;
// 	sor.setInputCloud (this->cloud);
// 	sor.setLeafSize (10, 10, 10);
// 	sor.filter (*cloud_filtered);
  
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (this->cloud);
	cluster_indices.clear();
	cluster_clouds.clear();
	pcl::EuclideanClusterExtraction<PointT> ec;
	
	ec.setClusterTolerance (70); // 2cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	
	ec.setInputCloud (this->cloud);
  ec.extract (cluster_indices);
	 
	cv::Mat rgbd_image(480,640, CV_8UC3, cv::Scalar::all(0));
	
	//lets transform the image to opencv
	cout<<rgbMatrix.size()<<endl;
	for(int i=0; i<rgbMatrix.size(); i++)
	{
// 		std::cout<<"the first one: " <<i<<std::endl;
		int row = i/640;
		int column = i-(row*640);
		
		rgbd_image.at<cv::Vec3b>(row,column) = cv::Vec3b(rgbMatrix[i].blue, rgbMatrix[i].green, rgbMatrix[i].red);
	}
	
	 
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
    ss <<"capture_"<<saved_counter<< "_object_" << j;
		
	/////save rgbd 
	#ifdef SAVE_DATA		
		cv::Mat M(480,640,CV_8UC1, cv::Scalar::all(0));
		for (int i = 0; i<cloud_cluster->points.size(); i++)
		{
			QVec xy = innermodel->project("robot", QVec::vec3(cloud_cluster->points[i].x, cloud_cluster->points[i].y, cloud_cluster->points[i].z), "rgbd"); 

			if (xy(0)>=0 and xy(0) < 640 and xy(1)>=0 and xy(1) < 480 )
			{
				M.at<uchar> ((int)xy(1), (int)xy(0)) = 255;
			}
			else if (not (isinf(xy(1)) or isinf(xy(0))))
			{
				std::cout<<"Accediendo a -noinf: "<<xy(1)<<" "<<xy(0)<<std::endl;
			}
		}
			
			//dilate
			cv::Mat dilated_M, z;
			cv::dilate( M, dilated_M, cv::Mat(), cv::Point(-1, -1), 2, 1, 1 );
			
			//find contour
			vector<vector<cv::Point> > contours;
			vector<cv::Vec4i> hierarchy;
			
			cv::findContours( dilated_M, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
			
			/// Draw contours
			cv::Mat mask = cv::Mat::zeros( dilated_M.size(), CV_8UC3 );
	// 		int contour_index = 1;

	// 		cv::Scalar color = cv::Scalar( 0, 255, 0 );
	// 		cv::drawContours( drawing, contours, contour_index, color, 2, 8, hierarchy, 0, cv::Point() );
			
			cv::drawContours(mask, contours, -1, cv::Scalar(255, 255, 255), CV_FILLED);
			
			
		// let's create a new image now
		cv::Mat crop(rgbd_image.rows, rgbd_image.cols, CV_8UC3);

		// set background to green
		crop.setTo(cv::Scalar(255,255,255));
			
			rgbd_image.copyTo(crop, mask);
			
			normalize(mask.clone(), mask, 0.0, 255.0, CV_MINMAX, CV_8UC1);
			
			cv::namedWindow( "Display window2", cv::WINDOW_AUTOSIZE );// Create a window for display.
		cv::imshow( "Display window2", rgbd_image );
			cv::imwrite( "scene.png", rgbd_image );
			
			cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
		cv::imshow( "Display window", crop );

			cv::imwrite( ss.str() + ".png", crop );

			/////save rgbd end
			
		
		
	#endif

		InnerModelNode *parent = innermodel->getNode(QString::fromStdString("rgbd"));
		innermodel->newTransform(QString::fromStdString("marca"), QString::fromStdString("static") ,parent, 356.152, -289.623, 756.853, 0.32, 0, 0);

		QMat PP = innermodel->getTransformationMatrix("marca", "robot");
		
		for (unsigned int i=0; i<cloud_cluster->points.size(); i++)
		{
			QVec p1 = QVec::vec4(cloud_cluster->points[i].x,cloud_cluster->points[i].y, cloud_cluster->points[i].z, 1);
			QVec p2 = PP * p1;
			QVec p22 = p2.fromHomogeneousCoordinates();

			cloud_cluster->points[i].x=p22(0);
			cloud_cluster->points[i].y=p22(1);
			cloud_cluster->points[i].z=p22(2);
			cloud_cluster->points[i].r=cloud_cluster->points[i].r;
			cloud_cluster->points[i].g=cloud_cluster->points[i].g;
			cloud_cluster->points[i].b=cloud_cluster->points[i].b;
		}
		writer.write<PointT> (ss.str () + ".pcd", *cloud_cluster, false); 
		j++;
  }
  saved_counter++;
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
	
// 	cv::Mat depth_image(480,640, CV_32F, cv::Scalar::all(0));
// 
// 
// 	for(int i=0; i<distanceMatrix.size(); i++)
// 	{
// 
// 					int row = i/640;
// 					int column = i-(row*640);
// 					if (distanceMatrix[i]==NAN){
// 
// 									depth_image.at<float>(row,column) =0;
// 					}else{
// 
// 									depth_image.at<float>(row,column) = distanceMatrix[i];
// 					}
// 	}
// 	double min,max;
// 
// 	cv::minMaxIdx(distanceMatrix,&min,&max);
// 
// 	cout<<"Minimo: "<<min<<"Maximo: "<<max<<endl;
// 
// 	cv::Mat disparityImage;
// 	cv::convertScaleAbs(depth_image, disparityImage,255/max);
// 
// 	cv::namedWindow( "Depth image", cv::WINDOW_AUTOSIZE );//
// 	//Create a window for display.
// 	cv::imshow( "Depth image", disparityImage );
// 
// 	cv::imwrite( "Depth.png", disparityImage);
// 
// 	cv::Mat rgbd_image(480,640, CV_8UC3, cv::Scalar::all(0));
// 	
// 	//lets transform the image to opencv
// 	cout<<rgbMatrix.size()<<endl;
// 	for(int i=0; i<rgbMatrix.size(); i++)
// 	{
// // 		std::cout<<"the first one: " <<i<<std::endl;
// 		int row = i/640;
// 		int column = i-(row*640);
// 		
// 		rgbd_image.at<cv::Vec3b>(row,column) = cv::Vec3b(rgbMatrix[i].blue, rgbMatrix[i].green, rgbMatrix[i].red);
// 	}
// 	cv::imwrite( "rgbd.png", rgbd_image);
	
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
	cloud->width = 1;
	cloud->height = points_kinect.size();
	
	std::vector< int > index;
	removeNaNFromPointCloud (*cloud, *cloud, index);
	
	//lets make a copy to maintain the origianl cloud
	*original_cloud = *cloud;

	writer.write<PointT> ("out.pcd", *cloud, false);
	
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
 	//ading to local innermodel
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
