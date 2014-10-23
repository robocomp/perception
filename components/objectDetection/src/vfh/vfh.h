#ifndef _VFH_H
#define _VFH_H

#include <fstream>
#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>

#include <flann/flann.h>
#include <flann/io/hdf5.h>

#define VFH_FILES_EXTENSION ".pcd"

typedef std::pair<std::string, std::vector<float> > vfh_model;
typedef pcl::PointXYZRGB PointT;

class VFH
{
	std::string kdtree_idx_file_name;
	std::string training_data_h5_file_name;
	std::string training_data_list_file_name;
	
	std::vector<vfh_model> models;
	flann::Matrix<int> k_indices;
	flann::Matrix<float> k_distances;
	flann::Matrix<float> data;
public:
	
	//Loads vfh histogram
	bool loadHist (const boost::filesystem::path &path, vfh_model &vfh);
	
	//computes an vfh histogram over a point cloud
	void computeVFHistogram(pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs);
	
	//read a full directory and computes their vfh
	void readFilesAndComputeVFH (const boost::filesystem::path &base_dir);
	
	//loads all vfh models in a directory
	void loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension, std::vector<vfh_model> &models);
	
	//loads all vfh models found in a given directory
	void reloadVFH(std::string path_to_dir);
	
	/** \brief Load the list of file model names from an ASCII file
	* \param models the resultant list of model name
	* \param filename the input file name
	*/
	bool loadFileList(std::vector<vfh_model> &models, const std::string &filename);
	
	//laod training data
	void loadTrainingData();
	
	/** \brief Search for the closest k neighbors
	* \param index the tree
	* \param model the query model
	* \param k the number of neighbors to search for
	* \param indices the resultant neighbor indices
	* \param distances the resultant neighbor distances
	*/
	void nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model, 
				int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances);
	
	//guess with trained data
	void doTheGuess(const pcl::PointCloud<PointT>::Ptr object, std::vector<std::string> &guesses);
};

#endif
