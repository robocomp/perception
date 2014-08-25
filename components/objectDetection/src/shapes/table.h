#ifndef TABLE_H_
#define TABLE_H_

#include "basic/rectprism.h"
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <qmat/QMatAll>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZRGB PointT;

using namespace std;

class Table
{
  
public:
	
	Table();
	Table(const double Cx, const double Cy, const double Cz,
						 const double Rx, const double Ry, const double Rz,
						 const double Wx, const double Wy, const double Wz);
	Table(const boost::shared_ptr<RectPrism> board);
	
	//board management methods;
	inline void set_board_size(const double Wx, const double Wy, const double Wz) { board->set_size(QVec::vec3(Wx, Wy, Wz)); }
	inline void set_board_center(const double Cx, const double Cy, const double Cz) { board->set_center(QVec::vec3(Cx, Cy, Cz)); }
	inline void set_board_rotation(const double Rx, const double Ry, const double Rz) { board->set_rotation(QVec::vec3(Rx, Ry, Rz)); }
	
	inline QVec get_board_size () { return board->get_size(); }
	inline QVec get_board_center () { return board->get_center(); }
	inline QVec get_board_rotation () { return board->get_rotation(); }
	
	//RANSAC to point cloud board
	void fit_board_with_RANSAC(pcl::PointCloud<PointT>::Ptr cloud, const float threshold);
	
	bool check_point_inside(const float x, const float y, const float z) { return board->check_point_inside(QVec::vec3(x,y,z)); }
	
	
//   static double distance_p2p (double x1, double y1, double z1, double x2, double y2, double z2);
//   inline const QVec getCenter () { return center; }
//   inline const QVec getRotation () { return rotation; }
//   inline const QVec getWidth () { return QVec::vec3(Wx,Wy,Wz); }
// 
//   inline void setCenter ( const QVec center ) { this->center=center; }
//   inline void setRotation ( const QVec  rotation ) { this->rotation=rotation; }
//   inline void setWidth ( const QVec Width ) { this->Wx=Width(0);this->Wy=Width(1);this->Wz=Width(2); }
//   
//   QVec placePoint(const QVec &point);
//   uint8_t collisionVector(const QVec &point);
//   
//   double distance(const QVec &point);
  
private:
//   QVec center;
//   QVec rotation;
//   double Wx, Wy, Wz;
	const boost::shared_ptr<RectPrism> board;

};

#endif
