#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <flann/flann.h>
#include <flann/algorithms/dist.h>


#define FILES_EXTENSION ".pcd"

//Function that computes the Viewpoint Feature Histogram
void computeVFHistogram(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const boost::filesystem::path &filename)
{
	pcl::console::print_highlight ("Computing VFH for %s.\n", filename.string().c_str());
	//---compute normals---
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	//create normal estimation class, and pass the input cloud
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud);

	//Create empty kdetree representation, and pass it to the normal estimation object.
	//its content will be filled inside the object based on the given input.
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	//set radious of the neighbors to use (1 cm)
	ne.setRadiusSearch(10);
	//computing normals
	ne.compute(*cloud_normals);

	//---proceed to compute VFH---
	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308> ());

	//Create the VFH estimation class and pas the input to it
	pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud (cloud);
	vfh.setInputNormals (cloud_normals);

	//create an empty kdtree representation and pass it to the vfh estimation object
	//its content will be filled inside the object based on the given input.
	pcl::search::KdTree<pcl::PointXYZ>::Ptr vfhtree (new pcl::search::KdTree<pcl::PointXYZ> ());
	vfh.setSearchMethod (vfhtree);

	//compute the features
	vfh.compute (*vfhs);

	//save them to file
	pcl::PCDWriter writer;
	std::stringstream ss;
	ss << filename.branch_path().string() << "/" << boost::filesystem::basename(filename) << ".vfh";
	pcl::console::print_highlight ("writing %s\n", ss.str().c_str());
	writer.write<pcl::VFHSignature308> (ss.str(), *vfhs, false);
	
}

//Function that recursively reads all files and computes the VFH for them
void readFilesAndComputeVFH (const boost::filesystem::path &base_dir)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	//Recursively read all files and compute VFH
	for(boost::filesystem::directory_iterator it (base_dir); it!=boost::filesystem::directory_iterator (); ++it)
	{
		std::stringstream ss;
		ss << it->path();
		//if its a directory just call back the function
		if (boost::filesystem::is_directory (it->status()))
		{
			pcl::console::print_highlight ("Entering directory %s.\n", ss.str().c_str());
			//call rescursively our function
			readFilesAndComputeVFH(it->path());
		}
		//if not, go ahead and read and process the file
		if (boost::filesystem::is_regular_file (it->status()) && boost::filesystem::extension (it->path()) == FILES_EXTENSION )
		{
			if(pcl::io::loadPCDFile<pcl::PointXYZ> (it->path().string(), *cloud) == -1)
				PCL_ERROR ("Couldn't read the file %s.", it->path().string().c_str());
			else
			{
				//Finally compute the vfh and save it
				computeVFHistogram(cloud, *it);
			}
		}
	}
}


int main (int argc, char *argv[] )
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
	
	//Check params
	if ( argc != 2 )
	{
		std::cout<<"You need to specify exactly one parameter"<<std::endl;
		std::cout<<"It should be the path to the training data in pcd format"<<std::endl;
		return -1;
	}

	//----Reading poinclouds----
	const boost::filesystem::path base_dir = argv[1];
	//check if it's a valid path
	if(!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory(base_dir))
	{
		std::cout<<"Not a valid path to data, please make sure you specify a correct one"<<std::endl;
		return -1;
	}
	//get into the directory and compute VFHs.
	readFilesAndComputeVFH(argv[1]);

	//done ;)
	return 0;

}
