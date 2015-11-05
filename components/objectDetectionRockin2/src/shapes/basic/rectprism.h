#ifndef RECTPRISM_H_
#define RECTPRISM_H_

#include <vector>
#include <bitset>
#include <math.h>
#include <cassert>
#include <stdint.h>
#include <qmat/QMatAll>
#include "axis.h"
#include "codes.h"

//#include <CGAL/Nef_polyhedron_3.h>
//#include <CGAL/Homogeneous.h>
//#include <CGAL/squared_distance_3.h> 



//typedef CGAL::Homogeneous<double> Kernel;
// typedef CGAL::Extended_homogeneous<CGAL::Gmpz>  Kernel;
// typedef Kernel::Point_3 Point_3;
// typedef Kernel::Plane_3  Plane_3;
// typedef CGAL::Polyhedron_3<Kernel>  Polyhedron;
// typedef CGAL::Nef_polyhedron_3<Kernel> Nef_polyhedron_3;
// typedef Nef_polyhedron_3::Object_handle Object_handle;
// typedef Nef_polyhedron_3::Vertex_const_iterator  Vertex_const_iterator;

using namespace std;

class RectPrism
{
  
public:
  RectPrism();
  RectPrism(const QVec &center, const QVec &rotation, double Wx, double Wy, double Wz);
  inline const QVec get_center () { return center; }
  inline const QVec get_rotation () { return rotation; }
  inline const QVec get_size () { return QVec::vec3(Wx,Wy,Wz); }

  inline void set_center ( const QVec center ) { this->center=center; }
  inline void set_rotation ( const QVec  rotation ) { this->rotation=rotation; }
  inline void set_size ( const QVec Width ) { this->Wx=Width(0);this->Wy=Width(1);this->Wz=Width(2); }
  
  QVec placePoint(const QVec &point);
  QVec rotateNormal(const QVec &point);
  double getInternalDistance(const QVec point, const QVec normal);
  uint8_t collisionVector(const QVec &point);
  
  double distance(const QVec &point);
  
private:
  void naiveIncRotation(int index);
	
  QVec center;
  QVec rotation;
  double Wx, Wy, Wz;
  
};

#endif
