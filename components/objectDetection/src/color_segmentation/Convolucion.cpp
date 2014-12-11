
#include "Convolucion.h"


/* SinglePixelConvolution: Coge una imagen (niveles de gris), un kernel
y una posición, calcula la convolución en esa posición y devuelve el
nuevo pixel.*/

double SinglePixelConvolution(double **im, int x, int y,double **kernel,int kernelWidth,
                              int kernelHeight){

double pixel=0;
int i,j;

for (i=0;i<kernelWidth;i++){
   for(j=0;j<kernelHeight;j++){
       pixel=pixel+(im[x+i][y+j]*kernel[i][j]);
   }
}
return pixel;
}

/*Convolution2D:Coge una imagen y un kernel y aplica la convolucion sobre el
área de la imagen especificada por ancho y por alto*/

void Convolution2D(double **im,double **dest,int width,int height,
                   double **kernel,int kernelWidth,int kernelHeight){

int smallwidth,smallheight;
int i,j;

smallwidth=width-kernelWidth +1;
smallheight=height-kernelHeight+1;

for(i=0;i<smallwidth;i++){
  for(j=0;j<smallheight;j++){
     dest[i][j]=0;
  }
}

for(i=0;i<smallwidth;i++){
  for(j=0;j<smallheight;j++){
     dest[i][j]=SinglePixelConvolution(im,i,j,kernel,kernelWidth,kernelHeight);
  }
}
}

/*Convolution2DPadded: Coge una imagen y un kernel y aplica la convolucion sobre el
área de la imagen especificada por ancho y por alto,devolviendo una parte de la
imagen final*/

void Convolution2DPadded(double **im, double **large,int width,int height,
                   double **kernel,int kernelWidth,int kernelHeight){

int smallwidth,smallheight;
int i,j;
int top,left;
double **small;
cout<<"Me puedo cagar en todo"<<endl;

cout<<"width: "<<width<<" kernelWidth"<<kernelWidth<<" height: "<<height<<" kernelHeight: "<<kernelHeight<<endl;
smallwidth=width-kernelWidth +1;
smallheight=height-kernelHeight+1;
cout<<"Me puedo cagar en todo"<<endl;
top=kernelHeight/2;
left=kernelWidth/2;
cout<<"small: "<<small<<"smallwidth: "<<smallwidth<<" smallheight: "<<smallheight<<endl;
 small=new double* [smallwidth];
 for(i=0;i<smallwidth;i++)
   small[i]=new double[smallheight];
cout<<"Me puedo cagar en todo"<<flush<<endl;
Convolution2D(im,small,width,height,kernel,kernelWidth,kernelHeight);
cout<<"Me puedo cagar en todo"<<endl;


for(j=0;j<height;j++){
  for(i=0;i<width;i++){
     large[i][j]=0;
  }
}

for(j=0;j<smallheight;j++){
  for(i=0;i<smallwidth;i++){
     large[i+left][j+top]=small[i][j];
  }
}

 for(i=0;i<smallwidth;i++)
   delete(small[i]);
 delete(small);
}
