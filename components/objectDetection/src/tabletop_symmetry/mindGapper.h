/**
 * @file mindGapper.h
 * @brief Generates an object's full pointcloud from a partial view using symmetries w.r.t. a given resting plane
 */
#pragma once

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//-- OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * @class mindGapper
 */

typedef pcl::PointXYZRGB PointT;

class mindGapper {

 public:
  mindGapper();
  ~mindGapper();
  
  /**< Set table plane where the object rests and its partial cloud */
  void setTablePlane( std::vector<double> planeCoeff );


  /**< Set parameters for symmetry plane search */
  void setFittingParams( int _n = 6, int _m = 5, 
			 double _dj = 0.01, double _alpha = 20.0*M_PI / 180.0 );

  /**< Set Kinect params to calculate mirror errors */
  void setDeviceParams( int _width = 640, int _height = 480, 
			double _focal_length_in_pixels = 525,
			double _cx = 320, double _cy = 240 );

  /**< Generates mirroring cloud */
  int complete( pcl::PointCloud<PointT>::Ptr &_cloud );


  bool generate2DMask(  pcl::PointCloud<PointT>::Ptr _segmented_cloud,
			cv::Mat &_markMask,
			cv::Mat &_depthMask );
  cv::Mat get2DMask() { return mMarkMask; }
  


  // Debug functions
  bool viewMirror( int _ind );
  void printMirror( int _ind );
  pcl::PointCloud<PointT>::Ptr getCandidate(int _ind) { return mCandidates[_ind]; }
  bool viewInitialParameters();

 private:
  
  /** Helper functions for complete */
  pcl::PointCloud<PointT>::Ptr projectToPlane( pcl::PointCloud<PointT>::Ptr _cloud );
  pcl::PointCloud<PointT>::Ptr mirrorFromPlane( pcl::PointCloud<PointT>::Ptr _cloud,
						       Eigen::VectorXd _plane,
						       bool _joinMirrored = true );

  pcl::PointCloud<PointT>::Ptr mProjected;
  std::vector<pcl::PointCloud<PointT>::Ptr> mCandidates;
  std::vector<double> mDelta;
  std::vector<double> mDelta1;
  std::vector<double> mDelta2;
  pcl::PointCloud<PointT>::Ptr mCloud;

  /**< Variables */
  Eigen::VectorXd mPlaneCoeffs;
  int mN;
  int mM;
  double mDj;
  double mAlpha;
  
  // Mask variables
  double mF, mCx, mCy;
  cv::Mat mMarkMask; cv::Mat mDepthMask;
  cv::Mat mDTMask;
  int mWidth; int mHeight;

  Eigen::Vector3d mC;
  Eigen::Vector3d mEa, mEb;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
};
