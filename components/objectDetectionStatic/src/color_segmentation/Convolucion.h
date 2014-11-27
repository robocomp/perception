#include <iostream>

using namespace std;

/* SinglePixelConvolution: Coge una imagen (niveles de gris), un kernel
y una posición, calcula la convolución en esa posición y devuelve el
nuevo pixel.*/

double SinglePixelConvolution(double **im, int x, int y,double **kernel,int kernelWidth,
                              int kernelHeight);

/*Convolution2D:Coge una imagen y un kernel y aplica la convolucion sobre el
área de la imagen especificada por ancho y por alto*/

void Convolution2D(double **im,double **dest,int width,int height,
                   double **kernel,int kernelWidth,int kernelHeight);

/*Convolution2DPadded: Coge una imagen y un kernel y aplica la convolucion sobre el
área de la imagen especificada por ancho y por alto,devolviendo una parte de la
imagen final*/

void Convolution2DPadded(double **im, double **dest,int width,int height,
                   double **kernel,int kernelWidth,int kernelHeight);
