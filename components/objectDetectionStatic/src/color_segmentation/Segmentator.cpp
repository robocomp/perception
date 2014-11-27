#include "Segmentator.h"

Segmentator::Segmentator()
{
  threshold_0 = 50;
  threshold_1 = 100;
  image_to_segment = NULL;
}

void Segmentator::set_tresholds(int threshold_0, int threshold_1)
{
  this->threshold_0 = threshold_0;
  this->threshold_1 = threshold_1;
}

void Segmentator::set_image(cv::Mat *img)
{
  image_to_segment = new IplImage(*img);
}

cv::Mat Segmentator::segment()
{
	entrada.alto=image_to_segment->height;
  entrada.ancho=image_to_segment->width;
	
  entrada.im= new PIXEL_ENT* [entrada.alto];
  for(int i=0;i<entrada.alto;i++)
    entrada.im[i]= new PIXEL_ENT [entrada.ancho];
	cout<<"test02"<<endl;

	  
  for(int i=0; i<entrada.alto;i++)
  {
	  for(int j=0; j<entrada.ancho;j++)
	  {
		  entrada.im[i][j].R=image_to_segment->imageData[(i*(image_to_segment->widthStep))+j*3];
		  entrada.im[i][j].G=image_to_segment->imageData[(i*(image_to_segment->widthStep))+j*3+1];
		  entrada.im[i][j].B=image_to_segment->imageData[(i*(image_to_segment->widthStep))+j*3+2];
	  }
  }	

  RGB_HSI(&entrada);	

	cout<<"test03"<<endl;

  IMAGEN_CANNY canny;

  canny.alto=entrada.alto;
  canny.ancho=entrada.ancho;

  canny.im= new int* [canny.alto];
	  for(int i=0;i<canny.alto;i++)
		  canny.im[i]= new int [canny.ancho];
	cout<<"test04"<<endl;
  CANNY *cannyMap;
	  // Inicializacion
  cannyMap=new CANNY(3,0.1,5,20,canny.ancho,canny.alto);
	cout<<"test04.5"<<endl;
  for(int i=0;i<canny.alto;i++){
	  for(int j=0;j<canny.ancho;j++){
		  cannyMap->im_entrada[i][j]=(entrada.im[i][j].R+entrada.im[i][j].G+entrada.im[i][j].B)/3;
	  }
  }
  	cout<<"test04.6"<<flush<<endl;
  cannyMap->apply_canny();
  	cout<<"test04.65"<<flush<<endl;
  cannyMap->Dilatacion();

  	cout<<"test04.7"<<endl;
  for(int i=0;i<canny.alto;i++){
	  for(int j=0;j<canny.ancho;j++){
		  canny.im[i][j]=cannyMap->im_dilatada[i][j];
	  }
  }

  	cout<<"test04.8"<<endl;
	cout<<"test05"<<endl;

  for(int i=0; i<image_to_segment->height;i++){
	  for(int j=0; j<image_to_segment->width;j++){
		  image_to_segment->imageData[(i*(image_to_segment->widthStep))+j*3]=canny.im[i][j];
		  image_to_segment->imageData[(i*(image_to_segment->widthStep))+j*3+1]=canny.im[i][j];
		  image_to_segment->imageData[(i*(image_to_segment->widthStep))+j*3+2]=canny.im[i][j];
	  
	  }
  }	
	cout<<"test06"<<endl;

  BIP *bip;

  bip=new BIP(entrada.alto,entrada.ancho,15,threshold_0);
	cout<<"test07"<<endl;
  segmentada.alto=entrada.alto;
  segmentada.ancho=entrada.ancho;

  segmentada.im= new PIXEL_SEG* [entrada.alto];
	  for(int i=0;i<entrada.alto;i++)
		  segmentada.im[i]= new PIXEL_SEG [entrada.ancho];
	cout<<"test08"<<endl;

  bip->PerceptualSegmentation(entrada, threshold_0, threshold_1, canny,0.1, 1.0, &segmentada);

  cout<<bip->ContClase<<"-----"<<bip->ContClaseC<<endl;

  HSI_RGB(&segmentada);
	cout<<"test09"<<endl;
  for(int i=0; i<image_to_segment->height;i++)
  {
	  for(int j=0; j<image_to_segment->width;j++)
	  {
		  image_to_segment->imageData[(i*(image_to_segment->widthStep))+j*3]=segmentada.im[i][j].R;
		  image_to_segment->imageData[(i*(image_to_segment->widthStep))+j*3+1]=segmentada.im[i][j].G;
		  image_to_segment->imageData[(i*(image_to_segment->widthStep))+j*3+2]=segmentada.im[i][j].B;
	  }
  }	
  cout<<"test10"<<endl;
  return image_to_segment;
}

Segmentator::~Segmentator()
{
  
}