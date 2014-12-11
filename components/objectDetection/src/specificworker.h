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

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <qmat/QMatAll>

#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/don.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <flann/flann.h>
#include <flann/io/hdf5.h>

#include <innermodel/innermodel.h>

#include "shapes/table.h"
#include "fitting/pf_rect_prism_fitting.h"
#include "fitting/naive_rect_prism_fitting.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/features/normal_3d_omp.h>

#include "tabletop_symmetry/mirror.h"
#include "vfh/vfh.h"

#include "color_segmentation/Segmentator.h"

#include <AR/param.h>
#include <AR/ar.h>
#include <AR/arMulti.h>

#define DEBUG

/**
       \brief
       @author authorname
*/

class AprilTagModel
{
public:
	int id;
	float tx;
	float ty;
	float tz;
	float rx;
	float ry;
	float rz;
	QTime lastTime;
};

typedef std::map<int, AprilTagModel> TagModelMap;

typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> ColorHandlerT;

class SegmResult
{
public:
	SegmResult()
	{
		num = x = y = 0;
	}
	int32_t num;
	int32_t x, y;
};

inline int32_t imageIndex(const int32_t x, const int32_t y)
{
	return (x+y*640);
}

SegmResult getSegmentationInfo(uint8_t *image);
SegmResult getBlob(uint8_t *image, int32_t &x, int32_t &y);
void recursiveCall(uint8_t *image, const int32_t x, const int32_t y, SegmResult &r);

class SpecificWorker : public GenericWorker
{ 
	//table related stuff
	boost::shared_ptr<Table> table;
	boost::shared_ptr<RectPrism> box;
	PointT table_offset;
	PointT box_offset;	
	
	TagModelMap tagMap;
	QMutex *mutex;
	QMutex *point_cloud_mutex;
	QMutex *euclidean_mutex;
	
	//PCL data structures
	pcl::PointCloud<PointT>::Ptr cloud;
	pcl::PointCloud<PointT>::Ptr original_cloud;
	pcl::PointCloud<PointT>::Ptr segmented_cloud;
	pcl::PointCloud<PointT>::Ptr downsampled_cloud;
	pcl::PointIndices::Ptr ransac_inliers;
	
	pcl::PCDWriter writer;
	
	int saved_counter;
	
	//result
	string class_obj;
	string instance, instance_from_vfh;
	float posx, posy;
	float theta;
	
	//artoolkit shit:
	string pathtomarker_x;
	string pathtomarker_y;
	string pathtomarker_z;
	string pathtocameraparams;
	int id_marker;
	bool isSingle;
	///Multimarcas
	ARMultiMarkerInfoT  *mMarker;
	float probability;
	float ar_tx, ar_ty, ar_tz, ar_rx, ar_ry, ar_rz;
	float marca_tx, marca_ty, marca_tz, marca_rx, marca_ry, marca_rz;
	float size;
	
	cv::Mat yellow, pink, green;
	
	RoboCompRGBD::ColorSeq rgbMatrix;	
	RoboCompRGBD::depthType distanceMatrix;
	RoboCompRGBD::PointSeq points_kinect;
	RoboCompJointMotor::MotorStateMap h;
	RoboCompDifferentialRobot::TBaseState b;
	
	//color Segmentation
 	Segmentator seg;
	

	
	// Create the filtering object
	pcl::PassThrough<PointT> pass;
	
	cv::Mat rgb_image;
	
	//table data
	pcl::PointIndices::Ptr model_inliers_indices;
	pcl::PointCloud<PointT>::Ptr projected_plane;
	pcl::PointCloud<PointT>::Ptr cloud_hull;
	pcl::PointCloud<PointT>::Ptr cloud_to_normal_segment;
	pcl::PointIndices::Ptr prism_indices;
	

	
	//VFH
	boost::shared_ptr<VFH> vfh_matcher;
	std::vector<string> vfh_guesses;
	
	//Normal estimation stuff:
	pcl::NormalEstimationOMP<PointT, pcl::PointNormal> normal_estimation;
	static const double normal_scale = 60;
	///The minimum DoN magnitude to threshold by
//   static const double threshold = 0.2;
	
	//euclidean clustring
	std::vector<pcl::PointIndices> cluster_indices;
	std::vector<pcl::PointCloud<PointT>::Ptr> cluster_clouds;
	int object_to_show;
	
	RTMat viewpoint_transform;
	
	//action flags
	bool grabThePointCloudContinous_flag, getTableInliers_flag, projectTableInliers_flag, tableConvexHull_flag, extractTablePolygon_flag, getTableRANSAC_flag, euclideanClustering_flag, objectSelected_flag, continousMode_flag, normal_segmentation_flag, fitTheTable_flag;
	
	InnerModel *innermodel;
	
	//fitter
	naiveRectangularPrismFitting* naive_fitter;
	PfRectPrismFitting* pf_fitter;
	
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	
	//generic functions
	void aprilFitModel(const string& model);
	void fitModel(const string& model, const string& method);
	void fitPrismtoObjectNaive();
	void fitPrismtoObjectMCMC();
	void fitPrismtoObjectPf();
	void naive_fit_cb (const boost::shared_ptr<RectPrism>  &shape);
	void getInliers(const string& model);
	void ransac(const string& model);
	void projectInliers(const string& model);
	void convexHull(const string& model);
	void extractPolygon(const string& model);
	void normalSegmentation(const string& model);
	void passThrough();
	void statisticalOutliersRemoval();
	void segmentImage();
	void centroidBasedPose(float &x, float &y, float &theta);
	void grabTheAR();
	inline std::string getResult(const std::string &image = "", const std::string &pcd = "") { return "not implemented";};
	void grabThePointCloud(const  std::string &image = "", const  std::string &pcd = "");
	void setContinousMode(bool mode);
	//PC mirroring
	void mirrorPC();
	void mindTheGapPC();
	
	void euclideanClustering(int &num_clusters);
	void performEuclideanClustering();
	void showObject(int object_to_show);
	
	//vfh calls
	void reloadVFH();
	void loadTrainedVFH();
	void vfh(std::vector<string> &guesses);
	void surfHomography(std::vector<string> &guesses);
	
	void fitTheViewVFH();
	
	void reset();
	
	//generate a sinthetic cube
	pcl::PointCloud<PointT>::Ptr generate_sinthetic_cube(const int tx=0, const int ty=0, const int tz=0, const int Wx = 1000, const int Wy = 1000, const int Wz = 1000, const int res = 30);
	
	//utils
	static void threePointsToPlane (const PointT &point_a, 
                            const PointT &point_b, 
                            const PointT &point_c, 
                            const pcl::ModelCoefficients::Ptr plane);
	
	
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void newAprilTag(const tagsList& tags);

	void updatePointCloud();
	
	void aprilFitTheBox();
	void addTheBox(RoboCompInnerModelManager::Pose3D pose);
	void updateTheBox(RoboCompInnerModelManager::Pose3D pose);
	
	void aprilFitTheTable();
	void drawTheTable();
	void addTheTable(RoboCompInnerModelManager::Pose3D pose);
	void updateTheTable(RoboCompInnerModelManager::Pose3D pose);
	
	void drawThePointCloud(pcl::PointCloud<PointT>::Ptr cloud);
	
private:
	bool add_point_cloud_to_innermodels(const std::string &id, const RoboCompInnerModelManager::PointCloudVector &cloud);
	bool add_transform_to_innermodels(const std::string &item, const std::string &engine, const std::string &base, const RoboCompInnerModelManager::Pose3D &pose);
	bool add_mesh_to_innermodels(const std::string &item, const std::string &base, const RoboCompInnerModelManager::meshType &m);
	void update_transforms_on_innermodels (const std::string &item, const RoboCompInnerModelManager::Pose3D pose);
	
public slots:
 	void compute(); 

};

#endif
