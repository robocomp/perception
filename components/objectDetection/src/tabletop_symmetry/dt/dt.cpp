/**
 * @file dt.cpp
 */

//-- dt original headers
#include "dt.h"
#include "image.h"
#include "imutil.h"
#include "misc.h"

#include <iostream>

/**
 * @function matDT
 * @brief Generates the distance transform of an input gray image (255: pixel occupied, 0: empty)
 * @output: CV_32FC1 matrix with distance transform
 */
cv::Mat matDT( cv::Mat _img ) {

  // Load cv::Mat into a image type
  image<uchar> *img = new image<uchar>( _img.cols, _img.rows, true );
 
  for( int j = 0; j < _img.rows; ++j ) {
    for( int i = 0; i < _img.cols; ++i ) {
      if( _img.at<uchar>(j,i) == 255 ) {
	imRef( img, i,j ) = 1;
      } else {
	imRef( img, i,j ) = 0;
      }
    }
  }


  // Compute dt
  image<float> *img_dt = dt(img);

  // Take square roots
  for (int y = 0; y < img_dt->height(); y++) {
    for (int x = 0; x < img_dt->width(); x++) {
      imRef(img_dt, x, y) = sqrt(imRef(img_dt, x, y));
    }
  }

  // Store
  cv::Mat mat_dt = cv::Mat( _img.rows, _img.cols, CV_32FC1 );
  for( int j = 0; j < _img.rows; ++j ) {
    for( int i = 0; i < _img.cols; ++i ) {
      mat_dt.at<float>(j,i) = imRef( img_dt, i, j );
    }
  }

  delete img;
  delete img_dt;

  // Return
  return mat_dt;
}
