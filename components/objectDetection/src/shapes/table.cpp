#include "table.h"

Table::Table(): board(new RectPrism(QVec::vec3(0, 0, 0), QVec::vec3(0, 0, 0), 0, 0, 0))
{
	
}

Table::Table(const double Cx, const double Cy, const double Cz,
						 const double Rx, const double Ry, const double Rz,
						 const double Wx, const double Wy, const double Wz): board(new RectPrism())
{
	
	this->board->set_center(QVec::vec3(Cx,Cy,Cz));
	this->board->set_rotation(QVec::vec3(Rx,Ry,Rz));
	this->board->set_size(QVec::vec3(Wx,Wy,Wz));
	
}

Table::Table(const boost::shared_ptr<RectPrism> board): board(new RectPrism())
{ 
	
	this->board->set_center( board->get_center() );
	this->board->set_rotation( board->get_rotation() );
	this->board->set_size( board->get_size() );
}

//optimize board using ransac
void Table::fit_board_with_RANSAC(pcl::PointCloud<PointT>::Ptr cloud, const float threshold)
{  
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (threshold);
	
	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);
	
}