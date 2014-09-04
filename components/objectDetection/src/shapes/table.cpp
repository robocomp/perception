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

void Table::get_table_inliers(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<PointT>::Ptr inliers, const pcl::PointIndices::Ptr inliers_indices)
{
	std::cout<<"Getting Table Inliers"<<std::endl;

	QVec board_center = this->board->get_center();
	QVec board_rotation = this->board->get_rotation();

 	inliers_indices->indices.resize(cloud->points.size());
	inliers->points.resize(cloud->points.size());

	int j=0;
	int index=0;			
	
	RTMat translat_mat = RTMat(0,0,0, QVec::vec3(-board_center(0), -board_center(1), -board_center(2)));
	RTMat final_translat_mat = RTMat(board_rotation(0), board_rotation(1), board_rotation(2), QVec::vec3(0, 0, 0)).invert()* translat_mat;
	
	for (pcl::PointCloud<PointT>::iterator it = cloud->points.begin (); it < cloud->points.end (); ++it)
  {
		QVec placed_point = final_translat_mat*QVec::vec4((it->x), (it->y), (it->z) , 1);
// 		if (table->check_point_inside(it->x, it->y, it->z))
		QVec table_size = this->board->get_size();
		if (placed_point(0) < table_size(0) && placed_point(0) > (-table_size(0)) &&
			placed_point(1) < table_size(2) && placed_point(1) > -table_size(2) &&
			placed_point(2) < table_size(1) && placed_point(2) > (-table_size(1)))
		{
			inliers->points[j].x = (it->x);
			inliers->points[j].y = (it->y);
			inliers->points[j].z = (it->z);
// 			inliers->points[j].x = placed_point(0);
// 			inliers->points[j].y = placed_point(1);
// 			inliers->points[j].z = placed_point(2);
			inliers->points[j].r = 0;
			inliers->points[j].g = 0;
			inliers->points[j].b = 255;
			
			inliers_indices->indices[j]=index;
			j++;
		}
// 		cout<<placed_point(0)<<" "<<placed_point(1)<<" "<<placed_point(2)<<endl;
		index++;;
  }
  

  
  inliers->points.resize(j);
  inliers_indices->indices.resize(j);
	
}

void threePointsToPlane (const PointT &point_a, 
                            const PointT &point_b, 
                            const PointT &point_c, 
                            const pcl::ModelCoefficients::Ptr plane) 
{ 
  // Create Eigen plane through 3 points 
  Eigen::Hyperplane<float, 3> eigen_plane = 
    Eigen::Hyperplane<float, 3>::Through (point_a.getArray3fMap (), 
                                                          point_b.getArray3fMap (), 
                                                          point_c.getArray3fMap ()); 

  plane->values.resize (4); 

  for (int i = 0; i < plane->values.size (); i++) 
    plane->values[i] = eigen_plane.coeffs ()[i]; 
}

void Table::project_board_inliers(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointIndices::Ptr inliers_indices, const pcl::PointCloud<PointT>::Ptr plane_projected)
{
	
	QVec board_rotation = this->board->get_rotation();
	QVec board_translation = this->board->get_center();
	
	RTMat transform = RTMat(board_rotation(0), board_rotation(1), board_rotation(2), QVec::vec3( board_translation(0), board_translation(1), board_translation(2)));
	
	//lets get three points of the plane 
	QVec p1 = transform * QVec::vec4(0, 0, 0, 1);
	QVec p2 = transform * QVec::vec4(1, 0, 0, 1);
	QVec p3 = transform * QVec::vec4(0, 1, 0, 1);
	
// 	p1.print("p1");
// 	p2.print("p2");
// 	p3.print("p3");
	
	
// 	QVec normal = (p2-p1) * (p3-p1);
// 	normal = normal.normalize();
// 	float d = normal * p1;
// 	
// 	cout<<" A: "<<normal(0)<<" B: "<<normal(1)<<" C: "<<normal(2)<<" D: "<<d<<endl;
	
	PointT p_1,p_2, p_3;
	p_1.x = p1(0);
	p_1.y = p1(1);
	p_1.z = p1(2);
	
	p_2.x = p2(0);
	p_2.y = p2(1);
	p_2.z = p2(2);
	
	p_3.x = p3(0);
	p_3.y = p3(1);
	p_3.z = p3(2);

	pcl::ModelCoefficients::Ptr plane (new pcl::ModelCoefficients);
	
	threePointsToPlane(p_1, p_2, p_3, plane);
	
// 	cout<<"PCL0: "<<plane->values[0]<<endl;
// 	cout<<"PCL1: "<<plane->values[1]<<endl;
// 	cout<<"PCL2: "<<plane->values[2]<<endl;
// 	cout<<"PCL3: "<<plane->values[3]<<endl;
	
// 	this->cloud=model_inliers_cloud;
  
  cout<<"Model inliers!!: "<<inliers_indices->indices.size()<<endl;

	//Let's project inliers
	pcl::ProjectInliers<PointT> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setIndices (inliers_indices);
	proj.setInputCloud (cloud);
	proj.setModelCoefficients (plane);
	proj.filter (*plane_projected);

}

void Table::board_convex_hull(const pcl::PointCloud<PointT>::Ptr plane_projected, const pcl::PointCloud<PointT>::Ptr cloud_hull)
{
	//Let's construct a convex hull representation of the model inliers
	pcl::ConvexHull<PointT> chull;
	chull.setInputCloud(plane_projected);
	chull.reconstruct(*cloud_hull);
}

void Table::extract_table_polygon(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<PointT>::Ptr cloud_hull, QVec viewpoint, double height_min, double height_max, const pcl::PointCloud<PointT>::Ptr polygon_cloud)
{
	
 	pcl::ExtractPolygonalPrismData<PointT> prism_extract;
 	pcl::PointIndices::Ptr prism_indices (new pcl::PointIndices);
 	
 	prism_extract.setHeightLimits(height_min, height_max);
	prism_extract.setViewPoint(viewpoint(0), viewpoint(1), viewpoint(2));
 	prism_extract.setInputCloud(cloud);
 	prism_extract.setInputPlanarHull(cloud_hull);
 	prism_extract.segment(*prism_indices);
 	
	//let's extract the result
 	pcl::ExtractIndices<PointT> extract_prism_indices;
 	extract_prism_indices.setInputCloud(cloud);
 	extract_prism_indices.setIndices(prism_indices);
 	extract_prism_indices.filter(*(polygon_cloud));
	
}



float normAngle(float theta)
{
    while (theta > 2.*M_PIl)
        theta -= 2.*M_PIl;
    while (theta < -2.*M_PIl)
        theta += 2.*M_PIl;
    return theta;
}

QVec planeCoeffsToRot(const QVec &planeVector, const QVec &initialRots)
{
    QVec v = planeVector;

    // Compute R_x
    float RX = normAngle(atan2(v(1), -v(2)));
		if (abs(RX-initialRots(0)) >= M_PIl)
    {
        if (RX > 0)
            RX -= M_PIl;
        else
            RX += M_PIl;
    }

    // Compute R_y
    QMat m = Rot3D(-RX, 0, 0);
    v = m * v;
    const float RY = normAngle(-atan2(v(0), -v(2)));

    return QVec::vec3(RX, RY, initialRots(2));
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
	
	seg.setAxis(Eigen::Vector3f (0, 1, 0)); //perpendicular to y axis
	
	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);
	QVec qcoef = QVec::vec3(coefficients->values[0], coefficients->values[1], coefficients->values[2]) * (-1);
	
	qcoef.print("VECTOR NORMAL: ");
	
	QVec plane_rotation = board->get_rotation();
 	
	plane_rotation = planeCoeffsToRot(qcoef, plane_rotation);
	
	plane_rotation.print("plane_rotation:");
	
 	board->set_rotation(plane_rotation);
	
}