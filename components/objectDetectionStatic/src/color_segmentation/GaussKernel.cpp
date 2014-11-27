
#include "GaussKernel.h"

/* Funciones para generar el kernel del filtro de Gauss
y aplicarlo a una imagen*/

#include <math.h>

#include "Convolucion.h"

#define MIPI 3.141593

/* GaussianDiscrete2D: Calcula el valor discreto en x,y de la
distribución gaussiana en 2D.
Valores de entrada: theta:valor de theta para la distribución
                        gaussiana.
                    x,y:puntos a los que calcular el valor discreto.
Salida: valor gaussiano discreto. */

double GaussianDiscrete2D(double theta, int x, int y){

double g=0;
double ySubPixel, xSubPixel;

for (ySubPixel=y-0.5;ySubPixel<y+0.6;ySubPixel+=0.1){
    for(xSubPixel=x-0.5;xSubPixel<x+0.6;xSubPixel+=0.1){
      g=g+((1/(2*M_PI*theta*theta))*exp(-1*(xSubPixel*xSubPixel+ySubPixel*ySubPixel)
                                            /(2*theta*theta)));
    }
}

g=g/121;
return g;
}

/* Gaussian 2D: Calcula algunos valores discretos de la distribución
gaussiana en 2D.
Entradas: theta:valor de theta para la distribución
                        gaussiana.
          size: número de valores discretos(píxeles) a calcular.
Salida: kernel:valores calculados.*/

void Gaussian2D(double **kernel, double theta,int size){

int i,j;
double sum;

for(j=0;j<size;j++){
  for(i=0;i<size;i++){
    kernel[i][j]=GaussianDiscrete2D(theta,i-(size/2),j-(size/2));
  }
}

sum=0;

for(j=0;j<size;j++){
  for(i=0;i<size;i++){
    sum=sum+kernel[i][j];
  }
}
}

/* Smooth: Coge una imagen y una distribución gaussiana, calcula el kernel ç
apropiado y lo aplica a la imagen para suavizarla*/

void Smooth(double **im, double **ims,int width,int height, int size, double theta){

double **kernel;
int i;
cout<<"g 01"<<flush<<endl;
 kernel=new double*[size];
      for (i=0;i<size;i++)
	kernel[i]=new double[size];
cout<<"g 01"<<flush<<endl;
Gaussian2D(kernel,theta,size);
cout<<"g 01"<<flush<<endl;
Convolution2DPadded(im,ims,width,height,kernel,size,size);
cout<<"g 01"<<flush<<endl;
 for(i=0;i<size;i++)
   delete(kernel[i]);
 delete(kernel);
}

void SmoothImage(int **im,int **dest,int w,int h,int size,double theta){

int grey,i,j;
double **dest1;
double **im1;
	cout<<"sm 01"<<flush<<endl;
 dest1=new double* [w];
 im1=new double* [w];
 for(i=0;i<w;i++){
   dest1[i]=new double[h];
   im1[i]=new double[h];
 }
	cout<<"sm 01"<<flush<<endl;
for (i=0;i<w;i++){
  for(j=0;j<h;j++){
    im1[i][j]=im[i][j];
   }
}
	cout<<"sm 01"<<flush<<endl;
Smooth(im1,dest1,w,h,size,theta);
cout<<"sm 01"<<flush<<endl;
for(i=0;i<w;i++){
  for(j=0;j<h;j++){
    grey=(int)dest1[i][j];
    if(grey>255)grey=255;
    if(grey<0)grey=0;
    dest[i][j]=grey;
  }
}
	cout<<"sm 01"<<flush<<endl;
 for(i=0;i<w;i++){
   delete(im1[i]);
   delete(dest1[i]);
 }cout<<"sm 01"<<flush<<endl;
 delete(im1);
 delete(dest1);
}
