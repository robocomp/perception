#ifndef TABLE_H_
#define TABLE_H_

#include "basic/rectprism.h"
#include <boost/shared_ptr.hpp>
#include <qmat/QMatAll>

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
	inline void set_board_size(const double Wx, const double Wy, const double Wz) { board->setWidth(QVec::vec3(Wx, Wy, Wz)); }
	inline void set_board_center(const double Cx, const double Cy, const double Cz) { board->setCenter(QVec::vec3(Cx, Cy, Cz)); }
	inline void set_board_rotation(const double Rx, const double Ry, const double Rz) { board->setRotation(QVec::vec3(Rx, Ry, Rz)); }
	
	inline QVec get_board_size () { return board->getWidth(); }
	inline QVec get_board_center () { return board->getCenter(); }
	inline QVec get_board_rotation () { return board->getRotation(); }
	
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
