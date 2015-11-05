#ifndef SEGMENTATOR_H
#define SEGMENTATOR_H

#include <stdio.h>
#include <algorithm>
#include <vector>
#include <iostream>


#include "Imagen.h"
#include "BIP.h"
#include "Canny.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
class Segmentator
{

  int threshold_0;
  int threshold_1;
  const IplImage *image_to_segment;	
  IMAGEN_ENT entrada;	
  IMAGEN_SEG segmentada;
  
public:  
  Segmentator();
  void set_tresholds(int threshold_0, int threshold_1);
  void set_image(cv::Mat *img);
  cv::Mat segment();
  ~Segmentator();
  
};

#endif