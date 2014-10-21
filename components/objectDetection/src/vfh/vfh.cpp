#include "vfh.h"

bool VFH::loadHist (const boost::filesystem::path &path, vfh_model &vfh)
{
  int vfh_idx;
  // Load the file as a PCD
  try
  {
    pcl::PCLPointCloud2 cloud;
    int version;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    pcl::PCDReader r;
    int type; unsigned int idx;
    r.readHeader (path.string (), cloud, origin, orientation, version, type, idx);

    vfh_idx = pcl::getFieldIndex (cloud, "vfh");
    if (vfh_idx == -1)
      return (false);
    if ((int)cloud.width * cloud.height != 1)
      return (false);
  }
  catch (pcl::InvalidConversionException e)
  {
    return (false);
  }

  // Treat the VFH signature as a single Point Cloud
  pcl::PointCloud <pcl::VFHSignature308> point;
  pcl::io::loadPCDFile (path.string (), point);
  vfh.second.resize (308);

  std::vector <pcl::PCLPointField> fields;
  pcl::getFieldIndex (point, "vfh", fields);

  for (size_t i = 0; i < fields[vfh_idx].count; ++i)
  {
    vfh.second[i] = point.points[0].histogram[i];
  }
  vfh.first = path.string ();
  return (true);
}

//Function that computes the Viewpoint Feature Histogram
void VFH::computeVFHistogram(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const boost::filesystem::path &filename)
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
	ne.setRadiusSearch(0.01);
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
	ss << filename.branch_path().string() << "/" << boost::filesystem::basename(filename) << ".vfh.pcd";
	pcl::console::print_highlight ("writing %s\n", ss.str().c_str());
	writer.write<pcl::VFHSignature308> (ss.str(), *vfhs, false);
	
}

//Function that recursively reads all files and computes the VFH for them
void VFH::readFilesAndComputeVFH (const boost::filesystem::path &base_dir)
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
        if (boost::filesystem::is_regular_file (it->status()) && boost::filesystem::extension (it->path()) == VFH_FILES_EXTENSION )
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

void VFH::loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension, 
                   std::vector<vfh_model> &models)
{
  if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
    return;

  for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
  {
    if (boost::filesystem::is_directory (it->status ()))
    {
      std::stringstream ss;
      ss << it->path ();
      pcl::console::print_highlight ("Loading %s (%lu models loaded so far).\n", ss.str ().c_str (), (unsigned long)models.size ());
      loadFeatureModels (it->path (), extension, models);
    }
    if(boost::filesystem::is_regular_file (it->status ()))
			std::cout<<boost::filesystem::extension (it->path ())<<" "<<extension<<std::endl;
		if(boost::filesystem::extension (it->path ()) == extension)
			std::cout<<"YES"<<std::endl;
		
    if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
    {
            std::cout<<"in"<<std::endl;
      vfh_model m;
      if (loadHist (base_dir / it->path ().filename (), m))
        models.push_back (m);
    }
  }
}

void VFH::loadVFH(std::string path_to_dir)
{
	
	std::string kdtree_idx_file_name = "kdtree.idx";
	std::string training_data_h5_file_name = "training_data.h5";
	std::string training_data_list_file_name = "training_data.list";
	
	std::vector<vfh_model> models;
	
	std::string model_directory (path_to_dir);
	
	// Load the model histograms
	loadFeatureModels (model_directory, ".vfh", models);
  pcl::console::print_highlight ("Loaded %d VFH models. Creating training data %s/%s.\n", 
      (int)models.size (), training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
	
	 flann::Matrix<float> data (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());

  for (size_t i = 0; i < data.rows; ++i)
    for (size_t j = 0; j < data.cols; ++j)
      data[i][j] = models[i].second[j];

  // Save data to disk (list of models)
  flann::save_to_file (data, training_data_h5_file_name, "training_data");
  std::ofstream fs;
  fs.open (training_data_list_file_name.c_str ());
  for (size_t i = 0; i < models.size (); ++i)
    fs << models[i].first << "\n";
  fs.close ();
 
  // Build the tree index and save it to disk
  pcl::console::print_error ("Building the kdtree index (%s) for %d elements...\n", kdtree_idx_file_name.c_str (), (int)data.rows);
  flann::Index<flann::ChiSquareDistance<float> > index (data, flann::LinearIndexParams ());
  //flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
  index.buildIndex ();
  index.save (kdtree_idx_file_name);
  delete[] data.ptr ();
	
}
