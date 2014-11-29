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
void VFH::computeVFHistogram(pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs)
{
	//---compute normals---
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	//create normal estimation class, and pass the input cloud
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	ne.setInputCloud (cloud);

	//Create empty kdetree representation, and pass it to the normal estimation object.
	//its content will be filled inside the object based on the given input.
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
	ne.setSearchMethod (tree);
	//set radious of the neighbors to use (1 cm)
	ne.setRadiusSearch(10);
	//computing normals
	ne.compute(*cloud_normals);

	//---proceed to compute VFH---

	//Create the VFH estimation class and pas the input to it
	pcl::VFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud (cloud);
	vfh.setInputNormals (cloud_normals);

	//create an empty kdtree representation and pass it to the vfh estimation object
	//its content will be filled inside the object based on the given input.
	pcl::search::KdTree<PointT>::Ptr vfhtree (new pcl::search::KdTree<PointT> ());
	vfh.setSearchMethod (vfhtree);

	//compute the features
	vfh.compute (*vfhs);
}

//Function that recursively reads all files and computes the VFH for them
void VFH::readFilesAndComputeVFH (const boost::filesystem::path &base_dir)
{
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308> ());
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
			if(pcl::io::loadPCDFile<PointT> (it->path().string(), *cloud) == -1)
				PCL_ERROR ("Couldn't read the file %s.", it->path().string().c_str());
			else
			{
				pcl::console::print_highlight ("Computing VFH for %s.\n", boost::filesystem::path(*it).string().c_str());
				
				//Finally compute the vfh and save it
				computeVFHistogram(cloud, vfhs);
				
				//save them to file
				pcl::PCDWriter writer;
				std::stringstream ss;
				ss <<boost::filesystem::path(*it).branch_path()<< "/" << boost::filesystem::basename(*it) << ".vfh";
				pcl::console::print_highlight ("writing %s\n", ss.str().c_str());
				writer.write<pcl::VFHSignature308> (ss.str(), *vfhs, false);
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

void VFH::reloadVFH(std::string path_to_dir)
{
	
	kdtree_idx_file_name = "kdtree.idx";
	training_data_h5_file_name = "training_data.h5";
	training_data_list_file_name = "training_data.list";
	
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

	std::cout<<"Saving data to disk"<<std::endl;
  // Save data to disk (list of models)
  flann::save_to_file (data, training_data_h5_file_name, "training_data");
  std::ofstream fs;
  fs.open (training_data_list_file_name.c_str ());
  for (size_t i = 0; i < models.size (); ++i)
	{
		std::cout<<models[i].first<<std::endl;
    fs << models[i].first << "\n";
	}
  fs.close ();
 
  // Build the tree index and save it to disk
  pcl::console::print_error ("Building the kdtree index (%s) for %d elements...\n", kdtree_idx_file_name.c_str (), (int)data.rows);
  flann::Index<flann::ChiSquareDistance<float> > index (data, flann::LinearIndexParams ());
  //flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
  index.buildIndex ();
  index.save (kdtree_idx_file_name);
  delete[] data.ptr ();
	
}


bool VFH::loadFileList (std::vector<vfh_model> &models, const std::string &filename)
{
	std::ifstream fs;
	fs.open (filename.c_str ());
	if (!fs.is_open () || fs.fail ())
		return (false);

	std::string line;
	while (!fs.eof ())
	{
		getline (fs, line);
		if (line.empty ())
			continue;
		vfh_model m;
		m.first = line;
		models.push_back (m);
	}
	fs.close ();
	return (true);
}

void VFH::loadTrainingData()
{
	if (!boost::filesystem::exists ("training_data.h5") || !boost::filesystem::exists ("training_data.list"))
	{
		pcl::console::print_error ("Could not find training data models files %s and %s!\n", 
				training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
		return;
	}
	else
	{
		loadFileList (models, training_data_list_file_name);
		flann::load_from_file (data, training_data_h5_file_name, "training_data");
		pcl::console::print_highlight ("Training data found. Loaded %d VFH models from %s/%s.\n", 
				(int)data.rows, training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
	}
	
	if (!boost::filesystem::exists (kdtree_idx_file_name))
	{
		pcl::console::print_error ("Could not find kd-tree index in file %s!", kdtree_idx_file_name.c_str ());
		return;
	}
	else
	{
		flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams (kdtree_idx_file_name));
		index.buildIndex ();
	}
}

void VFH::nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model, 
				int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
{
	// Query point
	flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
	memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));

	indices = flann::Matrix<int>(new int[k], 1, k);
	distances = flann::Matrix<float>(new float[k], 1, k);
	index.knnSearch (p, indices, distances, k, flann::SearchParams (512));
	delete[] p.ptr ();
}
void VFH::doTheGuess(const pcl::PointCloud<PointT>::Ptr object, std::vector<std::string> &guesses)
{

	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308> ());
	computeVFHistogram(object, vfhs);
	vfh_model histogram;
	
	pcl::PointCloud <pcl::VFHSignature308> point;

	histogram.second.resize (308);

	std::vector <pcl::PCLPointField> fields;
	int vfh_idx = pcl::getFieldIndex (*vfhs, "vfh", fields);

	for (size_t i = 0; i < fields[vfh_idx].count; ++i)
	{
		histogram.second[i] = vfhs->points[0].histogram[i];
		std::cout<<histogram.second[i]<<std::endl;
	}
	histogram.first = "cloud_from_object.vfh";

	//let look for the match
	flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams ("kdtree.idx"));

	index.buildIndex ();
	nearestKSearch (index, histogram, 16, k_indices, k_distances);
	
	guesses.clear();
	
	pcl::console::print_highlight ("The closest 16 neighbors are:\n");
	for (int i = 0; i < 16; ++i)
	{
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid (*object, centroid);
		std::cerr<<centroid[0]<<centroid[1]<<centroid[2]<<centroid[3]<<std::endl;
		pcl::console::print_info ("    %d - %s (%d) with a distance of: %f\n", 
				i, models.at (k_indices[0][i]).first.c_str (), k_indices[0][i], k_distances[0][i]);
		guesses.push_back(models.at (k_indices[0][i]).first);
	}
}