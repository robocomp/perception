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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
,cloud(new pcl::PointCloud<PointT>)
,ransac_inliers (new pcl::PointIndices)
,projected_plane(new pcl::PointCloud<PointT>)
,cloud_hull(new pcl::PointCloud<PointT>)
,prism_indices (new pcl::PointIndices)
,rgb_image(480,640, CV_8UC3, cv::Scalar::all(0))
,color_segmented(480,640, CV_8UC3, cv::Scalar::all(0))
,table(new Table())
,vfh_matcher(new VFH())

{
	//let's set the sizes
	table->set_board_size(500,30,500);
        cout<<"starting the specific worker me cago en todo"<<endl;
        
        innermodel = new InnerModel("/home/robocomp/robocomp/components/perception/etc/genericPointCloud.xml");
        
        viewpoint_transform = innermodel->getTransformationMatrix("robot", "rgbd_t");
        
        marca_tx = marca_ty = marca_tz = marca_rx = marca_ry = marca_rz = 0;
        
        image_pub = nh.advertise<sensor_msgs::Image> ("/rockin/ursusteam/rgb/image", 1, true);
	pcd_pub = nh.advertise<sensor_msgs::PointCloud2> ("/rockin/ursusteam/depth_0/pointcloud", 1, true);
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

void SpecificWorker::readThePointCloud(const string &image, const string &pcd)
{
    rgb_image = cv::imread(image);
    
    if(! rgb_image.data )                              // Check for invalid inpute
    {
        cout <<  "Could not open or find the image rgb.png" << std::endl ;
    }
    
    if (pcl::io::loadPCDFile<PointT> (pcd, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file pcd.pcd \n");
    }
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
    ros::spin();  
// 	trylibrostime
}


void SpecificWorker::grabTheAR()
{

}

void SpecificWorker::aprilFitModel(const string &model)
{

}

void SpecificWorker::segmentImage()
{
	#ifdef DEBUG
	cv::imwrite("nosegmentada.png",rgb_image);
	std::cout<<"setting image"<<std::endl;
	#endif
	
	segmentator.set_image(&rgb_image);
	
	#ifdef DEBUG
	std::cout<<"Segmenting image"<<std::endl;
	#endif
	
	
	segmentator.set_tresholds(100, 150);
	color_segmented = segmentator.segment();
	
	#ifdef DEBUG
	std::cout<<"Segmented"<<std::endl;
	cv::imwrite("Segmentada.png",color_segmented);
	cv::Mat yellow, pink, green;
	cv::inRange(color_segmented, cv::Scalar(0, 150, 150), cv::Scalar(80, 255, 255), yellow);
	cv::imwrite("yellow.png",yellow);
	cv::inRange(color_segmented, cv::Scalar(65, 15, 125), cv::Scalar(150, 100, 255), pink);
	cv::imwrite("pink.png",pink);
	cv::inRange(color_segmented, cv::Scalar(25, 75, 50), cv::Scalar(106, 255, 150), green);
	cv::imwrite("green.png",green);
	#endif
}

void SpecificWorker::mindTheGapPC()
{

}

void SpecificWorker::getResult(const string &image, const string &pcd, detectionResult &detection )
{
    grabThePointCloud(image, pcd);
    
    //publish imageen
    cv_bridge::CvImage out_msg;
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3; // Or whatever
    out_msg.image = rgb_image; // Your cv::Matencoding
    image_pub.publish(out_msg.toImageMsg());
    
    //publish point cloud
    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(*cloud, cloud2);
    pcd_pub.publish(cloud2);
    
    detection.classDetected = "a";
    detection.instance = "a1";
    detection.x = 1.5;
    detection.y = 2.5;
    detection.theta = 3.5;
    
    
    
    
}

void SpecificWorker::centroidBasedPose(float &x, float &y, float &theta)
{

}

void SpecificWorker::reloadVFH()
{
    vfh_matcher->reloadVFH("/home/spyke/robocomp/components/perception/data/Rockin_set");
}

void SpecificWorker::ransac(const string &model)
{
	table->fit_board_with_RANSAC( cloud, ransac_inliers, 15);
}

void SpecificWorker::euclideanClustering(int &numCluseters)
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
	ec.setMinClusterSize (200);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	
	ec.setInputCloud (this->cloud);
        ec.extract (cluster_indices);
	 
// 	cv::Mat rgb_image(480,640, CV_8UC3, cv::Scalar::all(0));
	
	//lets transform the image to opencv
// 	cout<<rgbMatrix.size()<<endl;
// 	for(int i=0; i<rgbMatrix.size(); i++)
// 	{
// // 		std::cout<<"the first one: " <<i<<std::endl;
// 		int row = i/640;
// 		int column = i-(row*640);
// 		
// 		rgb_image.at<cv::Vec3b>(row,column) = cv::Vec3b(rgbMatrix[i].blue, rgbMatrix[i].green, rgbMatrix[i].red);
// 	}
	
	 
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
                #ifdef SAVE_DATA	
 
                std::stringstream ss;
                ss <<"capture_object_" << j;
		
                /////save rgbd 

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
		cv::Mat crop(rgb_image.rows, rgb_image.cols, CV_8UC3);

		// set background to green
		crop.setTo(cv::Scalar(255,255,255));
			
			rgb_image.copyTo(crop, mask);
			
			normalize(mask.clone(), mask, 0.0, 255.0, CV_MINMAX, CV_8UC1);
			
			cv::namedWindow( "Display window2", cv::WINDOW_AUTOSIZE );// Create a window for display.
		cv::imshow( "Display window2", rgb_image );
			cv::imwrite( "scene.png", rgb_image );
			
			cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
		cv::imshow( "Display window", crop );

			cv::imwrite( ss.str() + ".png", crop );

			/////save rgbd end
			
		
		
	#endif

// 		InnerModelNode *parent = innermodel->getNode(QString::fromStdString("rgbd"));
// 		innermodel->newTransform(QString::fromStdString("marca"), QString::fromStdString("static") ,parent, 356.152, -289.623, 756.853, 0.32, 0, 0);
// 
// 		QMat PP = innermodel->getTransformationMatrix("marca", "robot");
// 		
// 		for (unsigned int i=0; i<cloud_cluster->points.size(); i++)
// 		{
// 			QVec p1 = QVec::vec4(cloud_cluster->points[i].x,cloud_cluster->points[i].y, cloud_cluster->points[i].z, 1);
// 			QVec p2 = PP * p1;
// 			QVec p22 = p2.fromHomogeneousCoordinates();
// 
// 			cloud_cluster->points[i].x=p22(0);
// 			cloud_cluster->points[i].y=p22(1);
// 			cloud_cluster->points[i].z=p22(2);
// 			cloud_cluster->points[i].r=cloud_cluster->points[i].r;
// 			cloud_cluster->points[i].g=cloud_cluster->points[i].g;
// 			cloud_cluster->points[i].b=cloud_cluster->points[i].b;
// 		}	
// 
// #ifdef SAVE_DATA	
// 		writer.write<PointT> (ss.str () + ".pcd", *cloud_cluster, false); 
// #endif
		j++;
        }

}

void SpecificWorker::passThrough()
{
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud (this->cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (marca_tz, marca_tz + 1000);
  //pass.setFilterLimitsNegative (true);
        
        pass.filter (*this->cloud);
	pass.setInputCloud (this->cloud);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (marca_tx, marca_tx + 1000);
        pass.setInputCloud (this->cloud);
	pass.filter (*this->cloud);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (marca_ty-200, marca_ty+1000);
	pass.setInputCloud (this->cloud);
	pass.filter (*this->cloud);
        #ifdef DEBUG
        timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	string pcdname =  "/home/robocomp/robocomp/components/perception/" + QString::number(ts.tv_sec).toStdString() + "_passThrough.pcd";
	printf("<%s>\n", pcdname.c_str());
	writer.write<PointT> ( pcdname, *cloud, false);
        #endif
        
        
}

void SpecificWorker::surfHomography(listType &guesses)
{

}

void SpecificWorker::fitTheViewVFH()
{

}

void SpecificWorker::showObject(const int numObject)
{

}

void SpecificWorker::convexHull(const string &model)
{
    table->board_convex_hull(projected_plane, cloud_hull);
    #ifdef DEBUG
    std::cout<<"Cloud hull size: "<<cloud_hull->size()<<std::endl;
    #endif
}

void SpecificWorker::mirrorPC()
{

}

void SpecificWorker::statisticalOutliersRemoval()
{
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud (this->cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*(this->cloud));
}

void SpecificWorker::loadTrainedVFH()
{
    	vfh_matcher->loadTrainingData();
	std::cout<<"Training data loaded"<<std::endl;
}

void SpecificWorker::reset()
{

}

void SpecificWorker::normalSegmentation(const string &model)
{

}

void SpecificWorker::getInliers(const string &model)
{

}

void SpecificWorker::vfh(listType &guesses)
{
    int object__to_show = 0;
    vfh_matcher->doTheGuess(cluster_clouds[object__to_show], vfh_guesses);
	guesses = vfh_guesses;
}

void SpecificWorker::grabThePointCloud(const string &image, const string &pcd)
{
	try
	{
	rgbd_proxy->getImage(rgbMatrix, distanceMatrix, points_kinect,  h, b);
	}
	catch(Ice::Exception e)
	{
	qDebug()<<"Error talking to rgbd_proxy: "<<e.what();
	}

	#ifdef DEBUG
		cout<<"SpecificWorker::grabThePointcloud rgbMatrix.size(): "<<rgbMatrix.size()<<endl;
	#endif
	
	for(unsigned int i=0; i<rgbMatrix.size(); i++)
	{
		int row = i/640;
		int column = i-(row*640);
		
		rgb_image.at<cv::Vec3b>(row,column) = cv::Vec3b(rgbMatrix[i].blue, rgbMatrix[i].green, rgbMatrix[i].red);
	}
	
		
	cloud->points.resize(points_kinect.size());
	for (unsigned int i=0; i<points_kinect.size(); i++)
	{
		memcpy(&cloud->points[i], &points_kinect[i],3*sizeof(float));
		cloud->points[i].r=rgbMatrix[i].red;
		cloud->points[i].g=rgbMatrix[i].green;
		cloud->points[i].b=rgbMatrix[i].blue;
	}
	cloud->width = 1;
	cloud->height = points_kinect.size();
	
	std::vector< int > index;
	removeNaNFromPointCloud (*cloud, *cloud, index);
	
	timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	string pcdname =  "/home/robocomp/robocomp/components/perception/" + QString::number(ts.tv_sec).toStdString() + ".pcd";
	printf("<%s>\n", pcdname.c_str());
	writer.write<PointT> ( pcdname, *cloud, false);
        
        string imagename = "/home/robocomp/robocomp/components/perception/" + QString::number(ts.tv_sec).toStdString() + ".png";
        cv::imwrite( imagename ,rgb_image);
  
}

void SpecificWorker::fitModel(const string &model, const string &method)
{

}

void SpecificWorker::setContinousMode(const bool &mode)
{

}

void SpecificWorker::projectInliers(const string &model)
{
        table->project_board_inliers(this->cloud, ransac_inliers, projected_plane);
        #ifdef DEBUG
        timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	string pcdname =  "/home/robocomp/robocomp/components/perception/" + QString::number(ts.tv_sec).toStdString() + "_projectInliers.pcd";
	printf("<%s>\n", pcdname.c_str());
	writer.write<PointT> ( pcdname, *cloud, false);
        #endif
}

void SpecificWorker::extractPolygon(const string &model)
{
	cout<<"CloudHull size: "<<cloud_hull->points.size()<<endl;
        cout<<"Cloud size: "<<cloud->points.size()<<endl;

	//table->extract_table_polygon(this->cloud, cloud_hull, QVec::vec3(viewpoint_transform(0,3), viewpoint_transform(1,3), viewpoint_transform(2,3)) , 20, 1500, prism_indices, this->cloud);
        QVec::vec3(viewpoint_transform(0,3), viewpoint_transform(1,3), viewpoint_transform(2,3)).print("Viewpoint: ");
        table->extract_table_polygon(this->cloud, cloud_hull, QVec::vec3(0,0,1320) , 20, 1500, prism_indices, this->cloud);        
        
        cout<<"Prism size: "<<prism_indices->indices.size()<<endl;
	cout<<"Point Cloud size: "<<this->cloud->points.size()<<endl;
        #ifdef DEBUG
        timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	string pcdname =  "/home/robocomp/robocomp/components/perception/" + QString::number(ts.tv_sec).toStdString() + "_extractPolygon.pcd";
	printf("<%s>\n", pcdname.c_str());
	writer.write<PointT> ( pcdname, *cloud, false);
        #endif
}

void SpecificWorker::newAprilTag(const tagsList &tags)
{

}






