/**
 * @file mirror.cpp
 * @brief Implementation of paper by Bohg, 2010: Mind the Gap: Robotic Grasping under Incomplete Observation
 * @author A. Huaman Quispe <ahuaman3@gatech.edu>
 * @date 2014/08/07
 */

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>

#include "mirror.h"
#include <tabletop_symmetry/dt/dt.h>


/**
 * @function mirror
 * @brief Constructor 
 */
Mirror::Mirror() :
  mCloud( new pcl::PointCloud<PointT>() ),
  mProjected( new pcl::PointCloud<PointT>() ){
}

/**
 * @function ~Mirror
 * @brief Destructor 
 */
Mirror::~Mirror() {

}

/**
 * @function setTablePlane
 * @brief Set resting plane and object to complete based on symmetry
 */
void Mirror::setTablePlane( std::vector<double> _planeCoeffs ) {
  
  mPlaneCoeffs.resize( _planeCoeffs.size() );
  for( int i = 0; i < _planeCoeffs.size(); ++i ) {
    mPlaneCoeffs(i) = _planeCoeffs[i];
  }

  // Normalize (a,b,c), in case it has not been done already
  double norm = sqrt( pow(mPlaneCoeffs[0],2) + pow(mPlaneCoeffs[1],2) + pow(mPlaneCoeffs[2],2) );
  mPlaneCoeffs = mPlaneCoeffs / norm;
}


/**
 * @function setFittingParams
 * @brief n: Distance steps, m: Orientation steps, dj: Distance step size, alpha: +- rotation step size
 */
void Mirror::setFittingParams( int _n, int _m, 
				   double _dj, double _alpha ) {
  mN = _n;
  mM = _m;
  mDj = _dj;
  mAlpha = _alpha;
}

/**
 * @function setDeviceParams
 * @brief Set parameters of Kinect to calculate fitness functions for optimization process
 */
void Mirror::setDeviceParams( int _width, int _height, 
				  double _focal_length_in_pixels,
				  double _cx, double _cy ) {
  
  mWidth = _width; 
	mHeight = _height;
  mF = _focal_length_in_pixels;
  mCx = _cx; mCy = _cy;
}


/**
 * @function complete
 * @brief Use symmetries on plane to complete pointcloud 
 */
int Mirror::complete( pcl::PointCloud<PointT>::Ptr &_cloud ) {
  
  // 0. Store cloud, its visibility mask, its depth in a 2D matrix and the distance transform of it
  mCloud = _cloud;
  this->generate2DMask( mCloud,
			mMarkMask,
			mDepthMask );
  mDTMask = matDT( mMarkMask );

  // 1. Project pointcloud to plane
  mProjected = projectToPlane( mCloud );
  

  // 2. Find eigenvalues (first two,the last one will be zero since projected cloud is in 2D)
  pcl::PCA<PointT> pca;
  pca.setInputCloud( mProjected );
  Eigen::Vector3f eval = pca.getEigenValues();
  Eigen::Matrix3f evec = pca.getEigenVectors();

  Eigen::Vector4d c;
  pcl::compute3DCentroid( *mProjected, c );
  
  mC << c(0), c(1), c(2);
  mEa << (double) evec(0,0), (double) evec(1,0), (double) evec(2,0);
  mEb << (double) evec(0,1), (double) evec(1,1), (double) evec(2,1);


  // 3. Choose the eigen vector most perpendicular to the viewing direction as initial guess for symmetry plane
  Eigen::Vector3d v, s, s_sample;
  v = mC; // viewing vector from Kinect origin (0,0,0) to centroid of projected cloud (mC)

  // s: Line which is the intersection between symmetry and table planes
  if( abs(v.dot(mEa)) <= abs(v.dot(mEb)) ) { s = mEa; } 
  else { s = mEb; }

  
  // 4. Get candidate planes by shifting centroid and rotating initial guess
  Eigen::Vector3d Np; 
  double ang, dang;
  Eigen::VectorXd sp(4);
  Eigen::Vector3d np, cp, dir;

  Np << mPlaneCoeffs(0), mPlaneCoeffs(1), mPlaneCoeffs(2); 
  dang = 2*mAlpha / (double) (mM-1);
    
  for( int i = 0; i < mM; ++i ) {
        
    ang = -mAlpha +i*dang;
    s_sample = Eigen::AngleAxisd( ang, Np )*s;
    np = s_sample.cross( Np ); np.normalize();
    
    for( int j = 0; j < mN; ++j ) {

      if( np.dot(v) > -np.dot(v) ) { dir = np; } else { dir = -np; }
      cp = mC + dir*mDj*j;

      //Set symmetry plane coefficients
      sp << np(0), np(1), np(2), -1*np.dot( cp );

      // 5. Mirror
      mCandidates.push_back( mirrorFromPlane(_cloud, sp, false) );

    } // end for N    
  } // end for M


  // 6. Evaluate (optimization)
  mDelta.resize( mCandidates.size() );
  mDelta1.resize( mCandidates.size() );
  mDelta2.resize( mCandidates.size() );

  for( int i = 0; i < mCandidates.size(); ++i ) {

    cv::Mat mark_i = cv::Mat::zeros( mHeight, mWidth, CV_8UC3 );


    // Check inliers and outliers
    pcl::PointCloud<PointT>::iterator it;
    int px, py; PointT P;
    
    int outOfMask = 0; int frontOfMask = 0;
    double delta_1 = 0; double delta_2 = 0;
    for( it = mCandidates[i]->begin(); it != mCandidates[i]->end(); ++it ) {
      P = (*it);
      px = (int)( mF*(P.x / P.z) + mCx );
      py = (int)( mF*(P.y / P.z) + mCy );

//       if( (px < 0 || px >= mWidth) or ( py < 0 || py >= mHeight ) )
// 			{
// 				printf("%f %f %f\n", P.x, P.y, P.z);
// 				printf("mF: %f\n", mF);
// 				printf("mCx: %f\n", mCx);
// 				printf("mCy: %f\n", mCy);
// 				printf("mWidth: %d\n", mWidth);
// 				printf("mHeight: %d\n", mHeight);
// 				std::cout<<"hallo2: "<<px<<" "<< py<<std::endl; return -1;
// 
// 			}
      

      // MEASURE 1: OUT-OF-MASK PIXELS DISTANCE TO CLOSEST MASK PIXEL
      if( mMarkMask.at<uchar>(py,px) != 255 ) {
	outOfMask++;
	delta_1 += mDTMask.at<float>(py,px);
      }

      // MEASURE 2: IN-MASK PIXELS IN FRONT OF VISIBLE PIXELS
      else {
	double d = P.z - (double)(mDepthMask.at<float>(py,px));
	if( d < 0 ) {
	  frontOfMask++;
	  delta_2 += -d;
	}

      }
 

    } // end for it
    
    // Expected values
    mDelta1[i] = delta_1 / outOfMask;
    mDelta2[i] =  frontOfMask; //(delta_2 / frontOfMask);// * 1000.0 / 0.00780; // mmx pix/mm

    mDelta[i] = mDelta1[i] + mDelta2[i];

    std::cout << "["<<i<<"] Front of mask: "<< frontOfMask << std::endl;    
  } // for each candidate
 

  // Return cloud with highest metric
  int minInd = 0; double minVal = mDelta1[0];
  for( int i = 1; i < mDelta1.size(); ++i ) {
    if( mDelta1[i] < minVal ) { minVal = mDelta1[i]; minInd = i; }
  }

  int g = minInd / mN;
  std::cout << " Candidate with more overlapping (good) is: "<< minInd <<" from group "<<g<< std::endl;

  int oldMinInd = minInd;

  minInd = oldMinInd; minVal = mDelta2[minInd];
  for( int i = oldMinInd; i < (g+1)*mN; ++i ) {
    std::cout <<"[ "<<i<<"] D1: "<< mDelta1[i]<< " and D2: "<< mDelta2[i] << std::endl;
    
    // If next in group has exceedingly more out-of-masks pixels, stop searching
    if( mDelta1[i] - mDelta1[oldMinInd] > 0.5 ) {
      break;
    }

    if( mDelta2[i] < minVal ) { minVal = mDelta2[i]; minInd = i; }
  }
  
  std::cout << "Final  candidate with lowest front index is: "<< minInd << std::endl;

  _cloud = mCandidates[minInd];
  for( int i = 0; i < mCloud->points.size(); ++i ) {
	_cloud->points.push_back( mCloud->points[i] );
  }
	_cloud->width = _cloud->points.size();
	_cloud->height = 1;
  return minInd;
}



float min_distance (pcl::PointCloud<PointT>::Ptr _cloud, pcl::PointCloud<PointT>::Ptr _cloud2 )
{
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (_cloud);
	std::vector<int> point_idx_nkn_search(1);
  std::vector<float> point_nkn_squared_distance(1);
	float distance = -1;
	float point_nkn_distance = -1;
	for(int i=0;i<_cloud2->size(); ++i)
	{
		std::cout<<"DISTANCE "<<i<<" :"<<distance<<std::endl;
		tree->nearestKSearch(_cloud2->points[i], 1, point_idx_nkn_search, point_nkn_squared_distance );
		point_nkn_distance = sqrt(point_nkn_squared_distance[0]);
		if( distance == -1 or distance > point_nkn_distance)
			distance = point_nkn_distance;
	}
	
	return point_nkn_distance;
}

/**
 * @function centroidBasedComplete
 * @brief Project _cloud to plane (set by setTablePlane), results in a 2D cloud
 */
int Mirror::centroidBasedComplete( pcl::PointCloud<PointT>::Ptr &_cloud )
{
	
	if (pcl::io::loadPCDFile<PointT> ("/home/spyke/robocomp/components/perception/components/objectDetection/build/omniWheel.pcd", *_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file omniWheel.pcd \n");
    return (-1);
  }
	
	if(_cloud->points.size() < 0)
	{
		std::cout<<"Point cloud size must be > 0!! "<<std::endl;
		return -1;
	}
	
	std::cout<<"Size of cloud: "<<_cloud->points.size()<<std::endl;
	
	float min_calc_distance = -1;
	float max_z_distance  = 0;
	QVec min_vector = QVec::zeros(3);
	QVec max_z = QVec::zeros(3);
	QVec center = QVec::zeros(3);
	QVec qcentroid = QVec::zeros(3); 
	Eigen::Vector4f centroid;
	
	// 0- Lets look for min distance and max_z for centroid moving
	for( int i =0; i < _cloud->points.size(); ++i)
	{
		QVec point = QVec::vec3(_cloud->points[i].x, _cloud->points[i].y, _cloud->points[i].z);
		
		float norm = point.norm2();
/*		
		if ( min_calc_distance < 0 or norm < min_calc_distance )
		{
			min_calc_distance = norm;
			min_vector = point;
		}*/
		if ( max_z_distance < point[2] )
		{
			max_z_distance = point[2];
			max_z = point;
		}
	}
	
	std::cout<<"Max z distance: "<<max_z_distance<<std::endl;
	
	// 1- lets get the center to reflect and increment step
	compute3DCentroid (*_cloud, centroid);

	qcentroid = QVec::vec3(centroid(0), centroid(1), centroid(2));
	float multiplication_term = max_z_distance/qcentroid(1);

	qcentroid.print("qcentroid");
	
	center = qcentroid * multiplication_term;
		
	QVec inc = center - qcentroid;
	float incU = inc.norm2();
	
	//2- construct the mirror
	pcl::PointCloud<PointT>::Ptr mirrored_point_cloud (new pcl::PointCloud<PointT>());
	mirrored_point_cloud->points.resize(_cloud->points.size());
	for( int i = 0; i< _cloud->points.size(); ++i)
	{
		QVec point = QVec::vec3(_cloud->points[i].x, _cloud->points[i].y, _cloud->points[i].z);
		QVec mirrored_point = center - point + center  + (inc*2); //leaving it far away
		mirrored_point_cloud->points[i].x = mirrored_point(0);
		mirrored_point_cloud->points[i].y = mirrored_point(1);
		mirrored_point_cloud->points[i].z = mirrored_point(2);
		mirrored_point_cloud->points[i].r = _cloud->points[i].r;
		mirrored_point_cloud->points[i].g = _cloud->points[i].g;
		mirrored_point_cloud->points[i].b = _cloud->points[i].b;
	}
	
	std::cout<<"Mirrored_point_cloud_size: "<<mirrored_point_cloud->points.size()<<std::endl;
	
	//2- get the two clouds together
	min_calc_distance = min_distance(_cloud, mirrored_point_cloud);
	std::cout<<"Min dist: "<<min_calc_distance<<std::endl;
	
	while(min_calc_distance > 20)
	{
		std::cout<<"Lets get the point_clouds"<<std::endl;
		//lets get them closer
		for(int i = 0; i< mirrored_point_cloud->points.size(); i++)
		{
			std::cout<<mirrored_point_cloud->points.size()<<std::endl;
			QVec point = QVec::vec3(mirrored_point_cloud->points[i].x,mirrored_point_cloud->points[i].y,mirrored_point_cloud->points[i].z);
			point = point - (incU*min_calc_distance);
			mirrored_point_cloud->points[i].x = point(0);
			mirrored_point_cloud->points[i].y = point(1);
			mirrored_point_cloud->points[i].z = point(2);
		}
		std::cout<<"About to get min dist"<<std::endl;
		min_calc_distance = min_distance(_cloud, mirrored_point_cloud);
		std::cout<<"Min dist: "<<min_calc_distance<<std::endl;
	}
	
	//lets push_back result to the original pc
	*_cloud += *mirrored_point_cloud;
	
	
}


/**
 * @function projectToPlane
 * @brief Project _cloud to plane (set by setTablePlane), results in a 2D cloud
 */
pcl::PointCloud<PointT>::Ptr Mirror::projectToPlane( pcl::PointCloud<PointT>::Ptr _cloud ) {

  // 0. Init
  pcl::PointCloud<PointT>::Ptr projected( new pcl::PointCloud<PointT>() );

  // 1. Project and store
  pcl::PointCloud<PointT>::iterator it;
  PointT p; double a;


  // Plane equation: c0*x + c1*y + c2*z + c3 = 0
  // Original 3d point P will be projected (P') into plane with normal [c0, c1, c2]
  // P' = -(c3 + c0*x + c1*y + c2*z) / (c0^2 + c1^2 + c2^2) 
  for( it = _cloud->begin(); it != _cloud->end(); ++it ) {

    p = (*it);
    a = -( mPlaneCoeffs(3) + mPlaneCoeffs(0)*p.x + mPlaneCoeffs(1)*p.y + mPlaneCoeffs(2)*p.z );

    PointT pp;
    pp.x = p.x + mPlaneCoeffs(0)*a;
    pp.y = p.y + mPlaneCoeffs(1)*a;
    pp.z = p.z + mPlaneCoeffs(2)*a;

    projected->points.push_back( pp );
  }

  projected->height = projected->points.size();
  projected->width = 1;

  return projected;
}

/**
 * @function mirrorFromPlane
 * @brief Mirror pointcloud around _plane 
 */
pcl::PointCloud<PointT>::Ptr Mirror::mirrorFromPlane( pcl::PointCloud<PointT>::Ptr _cloud,
								 Eigen::VectorXd _plane,
								 bool _joinMirrored ) {

  // 0. Init
  pcl::PointCloud<PointT>::Ptr mirrored( new pcl::PointCloud<PointT>() );

  // 1. Project and store
  pcl::PointCloud<PointT>::iterator it;
  PointT p; double a;


  // Plane equation: c0*x + c1*y + c2*z + c3 = 0
  // Original 3d point P will be projected (P') into plane with normal [c0, c1, c2]
  // P' = -(c3 + c0*x + c1*y + c2*z) / (c0^2 + c1^2 + c2^2) 
  for( it = _cloud->begin(); it != _cloud->end(); ++it ) {

    p = (*it);
    a = -( _plane(3) + _plane(0)*p.x + _plane(1)*p.y + _plane(2)*p.z );

    PointT mp;
    mp.x = p.x + 2*_plane(0)*a;
    mp.y = p.y + 2*_plane(1)*a;
    mp.z = p.z + 2*_plane(2)*a;

    mirrored->points.push_back( mp );
  }

  // If you want the output to have the whole (original + mirrored points)
  if( _joinMirrored ) {    
    for( it = _cloud->begin(); it != _cloud->end(); ++it ) {
      mirrored->points.push_back( *it );
    }
  }

  mirrored->height = mirrored->points.size();
  mirrored->width = 1;
  
  return mirrored;
}

/**
 * @function viewMirror
 */
bool Mirror::viewMirror( int _ind ) {

  if( _ind >= mCandidates.size() || _ind < 0 ) {
    return false; 
  }

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer("Mind Gap") );
  viewer->setBackgroundColor(0,0,0);
  viewer->addCoordinateSystem(1.0, 0 );
  viewer->initCameraParameters();

  // Original - GREEN, mirror - BLUE
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color( mCloud, 0, 255, 0 );
  pcl::visualization::PointCloudColorHandlerCustom<PointT> mirror_color( mCandidates[_ind], 0, 0, 255 );
  viewer->addPointCloud( mCandidates[_ind], mirror_color, "mirror_cloud" );
  viewer->addPointCloud( mCloud, cloud_color, "cloud" );
  
  while( !viewer->wasStopped() ) {
    viewer->spinOnce(100);
    boost::this_thread::sleep( boost::posix_time::microseconds(100000));
  }


  return true;

}

/**
 * @function printMirror
 */
void Mirror::printMirror( int _ind ) {

  if( _ind < 0 || _ind >= mCandidates.size() ) { std::cout << "NO PRINTING"<< std::endl; return; }

  pcl::PointCloud<PointT>::iterator it;
  int px, py; PointT P;
  int outside = 0; int front = 0; int behind = 0;

  cv::Mat mark_i = cv::Mat::zeros( mHeight, mWidth, CV_8UC3 );

  for( it = mCandidates[_ind]->begin(); 
       it != mCandidates[_ind]->end(); ++it ) {
    P = (*it);
    px = (int)( mF*(P.x / P.z) + mCx );
    py = (int)( mF*(P.y / P.z) + mCy );
    
    // If outside: YELLOW
    if( mMarkMask.at<uchar>(py,px) != 255 ) {
      cv::Vec3b col(0,255,255);
      mark_i.at<cv::Vec3b>(py,px) = col;
      outside++;
    } 
    // If inside
    else {
      // If in front of visible  - MAGENTA
      if( (float)P.z < mDepthMask.at<float>(py,px) ) {
	cv::Vec3b col(255,0,255);
	mark_i.at<cv::Vec3b>(py,px) = col;
	front++;
      } else {
	// If behind - CYAN
	cv::Vec3b col(255,255,0);
	mark_i.at<cv::Vec3b>(py,px) = col;
	behind++;
      }
    }

  } // end for it

    char name[50];
    sprintf( name, "candidate_%d.png", _ind );
    imwrite( name, mark_i );

}


/**
 * @function viewInitialParameters
 * @brief View projected cloud, centroid and 2 eigenvalues Ea and Eb
 */
bool Mirror::viewInitialParameters() {

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer("Initial") );
  viewer->setBackgroundColor(0,0,0);
  viewer->addCoordinateSystem(1.0, 0 );
  viewer->initCameraParameters();

  // Original green, mirror blue
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color( mCloud, 0, 255, 0 );
  pcl::visualization::PointCloudColorHandlerCustom<PointT> projected_color( mProjected, 0, 0, 255 );
  viewer->addPointCloud( mProjected, projected_color, "projected" );
  viewer->addPointCloud( mCloud, cloud_color, "cloud" );
  
  // Center red ball
  PointT c;
  c.x = mC(0); c.y = mC(1); c.z = mC(2);
  double r, g, b;
  r = 1; g = 0; b = 0;
  viewer->addSphere( c, 0.015, r, g, b, "centroid" );

  // Draw Eigen vectors: ea magenta, eb yellow
  PointT pea, peb;
  double l = 0.20;
  pea.x = c.x + mEa(0)*l;   pea.y = c.y + mEa(1)*l;   pea.z = c.z + mEa(2)*l;
  peb.x = c.x + mEb(0)*l;   peb.y = c.y + mEb(1)*l;   peb.z = c.z + mEb(2)*l;

  r = 1.0; g = 0.0; b = 1.0;
  viewer->addLine( c, pea, r, g, b, "ea", 0 );
  r = 1.0; g = 1.0; b = 0.0;
  viewer->addLine( c, peb,  r, g, b, "eb", 0 );

  while( !viewer->wasStopped() ) {
    viewer->spinOnce(100);
    boost::this_thread::sleep( boost::posix_time::microseconds(100000));
  }

  
  return true;

}



/**
 * @function generate2DMask
 * @brief Generates image with visible segmented pixels from _segmented_cloud, _depthMask stores the depth of each
 */
bool Mirror::generate2DMask( pcl::PointCloud<PointT>::Ptr _segmented_cloud,
				 cv::Mat &_markMask,
				 cv::Mat &_depthMask ) {

  _markMask = cv::Mat::zeros( mHeight, mWidth, CV_8UC1 );
  _depthMask = cv::Mat::zeros( mHeight, mWidth, CV_32FC1 );
  
  
  // Color the segmented pixels
  pcl::PointCloud<PointT>::iterator it;
  PointT P;
  int px; int py;
  
  for( it = _segmented_cloud->begin(); 
       it != _segmented_cloud->end(); ++it ) {
    P = (*it);
    px = (int)( mF*(P.x / P.z) + mCx );
    py = (int)( mF*(P.y / P.z) + mCy );

    if( px < 0 || px >= mWidth ) { return false; }
    if( py < 0 || py >= mHeight ) { return false; }
    

    _markMask.at<uchar>(py,px) = 255;
    _depthMask.at<float>(py,px) = (float)P.z;
  }
  

  return true;
}



