#ifndef _VFH_H
#define _VFH_H

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>

#include <flann/flann.h>
#include <flann/io/hdf5.h>

#define VFH_FILES_EXTENSION ".pcd"

typedef std::pair<std::string, std::vector<float> > vfh_model;

class VFH
{
public:
	//Loads vfh histogram
	bool loadHist (const boost::filesystem::path &path, vfh_model &vfh);
	
	//computes an vfh histogram over a point cloud
	void computeVFHistogram(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const boost::filesystem::path &filename);
	
	//read a full directory and computes their vfh
	void readFilesAndComputeVFH (const boost::filesystem::path &base_dir);
	
	//loads all vfh models in a directory
	void loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension, std::vector<vfh_model> &models);
	
	//loads all vfh models in a directory given
	void loadVFH(std::string path_to_dir);
};

#endif
