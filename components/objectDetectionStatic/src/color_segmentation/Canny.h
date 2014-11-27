
#ifndef CANNY_H
#define CANNY_H

#include <iostream>

using namespace std;

typedef struct PUNTO2{
int x;
int y;
}PUNTO2;

struct CANNY{
  int **im_entrada;
  int **im_lineas;
  unsigned char **im_dilatada;
  int kernel_size;
  float theta;
  int lowthresh;
  int highthresh;
  float scale;
  int offset;
  int height;
  int width;

  CANNY(int _kernel_size, float _theta, int _lowthresh, int _highthresh, int _width, int _height);
  ~CANNY();

  void apply_canny();
  void Dilatacion();
};

#endif
