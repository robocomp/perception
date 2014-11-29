/*Funciones para aplicar el operador de Canny a una imagen*/

#include<stack>
#include <math.h>

#include "GaussKernel.h"
#include "Canny.h"


CANNY::CANNY(int _kernel_size, float _theta, int _lowthresh, int _highthresh, int _width, int _height){

  kernel_size=_kernel_size;
  theta=_theta;
  lowthresh=_lowthresh;
  highthresh=_highthresh;
  height=_height;
  width=_width;
  scale=1.0;
  offset=255;

  register int i;

  im_entrada=new int* [height];
  im_lineas=new int* [height];
  im_dilatada=new unsigned char* [height];

  for(i=0;i<height;i++){
    im_entrada[i]=new int [width];
    im_lineas[i]=new int[width];
    im_dilatada[i]=new unsigned char[width];
  }
}

CANNY::~CANNY(){
  register int i;

  for(i=0;i<height;i++){
    delete(im_entrada[i]);
    delete(im_lineas[i]);
  }
  delete(im_entrada);
  delete(im_lineas);
}

int d_w,d_h;

  //Function to check which sector the line is in (see Machine Vision pg 171)
   int sector(double theta){
    int i=0;
    //Converting into degrees from radians, and moving to lie between 0 and 360
    theta = (theta*180)/M_PI;
    theta = theta + 270 ;
    //theta = theta - (theta div 360)*360;
    while((i*360)<theta){
       i++;
    }
    i=i-1;
    theta=theta - 360*i;


    if((theta >= 337.5) || (theta < 22.5) || ((theta >= 157.5) && (theta < 202.5))){
      return 0;
    }
    if(((theta >= 22.5) && (theta < 67.5)) || ((theta >=202.5) && (theta < 247.5))){
      return 1;
    }
    if(((theta >= 67.5) && (theta < 112.5)) || ((theta >=247.5) && (theta < 292.5))){
      return 2;
    }
    if(((theta >= 112.5) && (theta < 157.5)) || ((theta >= 292.5) && (theta < 337.5))){
      return 3;
    }

    return 0;
  }

  // Function to apply non maxima suppression to the image array
  int suppress(int **m_2d, int sector, int i, int j, int lowthresh){

    int tmp = m_2d[i][j];
    if (tmp < lowthresh) return 0;


    if(sector == 0){
      if((m_2d[i+1][j] >= tmp) || (m_2d[i-1][j] > tmp)){
	return 0;
      }
      else {
	return tmp;
      }
    }
    if(sector == 1){
      if((m_2d[i+1][j+1] >= tmp) || (m_2d[i-1][j-1] > tmp)){
	return 0;
      }
      else {
	return tmp;
      }
    }
    if(sector == 2){
      if((m_2d[i][j+1] >= tmp) || (m_2d[i][j-1] > tmp)){
	return 0;
      }
      else {
	return tmp;
      }
    }
    if(sector == 3){
      if((m_2d[i+1][j-1] >= tmp) || (m_2d[i-1][j+1] > tmp)){
	return 0;
      }
      else {
	return tmp;
      }
    }
    return 0;
  }

 /*The function track is called once a starting point for tracking has been
   *found. When this happens, this function follows all possible paths above
   *the threshold by placing unsearched paths on the stack. Each time a path
   *is looked at it's pixels are marked. This continues until the stack is
   *empty, at which point the new array of marked paths is returned.
   */

  void track(int **input, int  **marked, std::stack<int> &to_track,
			int thresh, int i, int j){

    //empty represents when the stack is empty
    bool empty = false;
    int a;
    int b;
    //Create a point to represent where to start the tracking from
    PUNTO2 current;
    current.x=i;
    current.y=j;

    //Push the initial point onto the stack
    to_track.push(current.y);
    to_track.push(current.x);
    while(!empty){
      // try{

	//Take the top pixel from the stack
	current.x =to_track.top(); 
	to_track.pop();
        current.y=to_track.top();
	to_track.pop();
        //to_track.pop();
	//Find it's co-ordinates
	a = current.x;
	b = current.y;
	//Now check neighbourhood and add to stack anything above thresh
	//Only done if pixel is currently unmarked
	if(marked[a][b] == 0){

	  //Try and track from each neighbouring point
	  if(a > 0 && b > 0){
	    if(input[a-1][b-1] > thresh){
	      current.x= (a-1);
              current.y= (b-1);
	      to_track.push(current.y);
              to_track.push(current.x);
	    }
	  }

	  if(b > 0){
	    if(input[a][b-1] > thresh){
	      current.x=(a);
              current.y= (b-1);
	      to_track.push(current.y);
              to_track.push(current.x);
	    }
	  }

	  if(a < (d_w-1) && b > 0){
	    if(input[a+1][b-1] > thresh){
	      current.x=(a+1);
               current.y=(b-1);
	      to_track.push(current.y);
              to_track.push(current.x);
	    }
	  }

	  if(a > 0){
	    if(input[a-1][b] > thresh){
	      current.x =(a-1);
              current.y=(b);
	      to_track.push(current.y);
              to_track.push(current.x);
	    }
	  }

	  if(a < (d_w-1)){
	    if(input[a+1][b] > thresh){
	      current.x = (a+1);
              current.y= (b);
	      to_track.push(current.y);
              to_track.push(current.x);
	    }
	  }

	  if(a > 0 && b < (d_h-1)){
	    if(input[a-1][b+1] > thresh){
	      current.x=(a-1);
              current.y= (b+1);
	      to_track.push(current.y);
              to_track.push(current.x);
	    }
	  }

	  if( b < (d_h-1)){
	    if(input[a][b+1] > thresh){
	      current.x=(a);
              current.y= (b+1);
	      to_track.push(current.y);
              to_track.push(current.x);
	    }
	  }

	  if(a < (d_w-1) && b < (d_h-1)){
	    if(input[a+1][b+1] > thresh){
	      current.x=(a+1);
              current.y= (b+1);
	      to_track.push(current.y);
              to_track.push(current.x);
	    }
	  }

	  //Mark this pixel as having been tracked from
	  marked[a][b] = 1;
	}
   if (to_track.size()==0){
        empty=true;
        }

    }

      //If stack empty, then set the empty flag to true
      // catch(EListError &e){
     	// }
	// catch (EAccessViolation &e){
	// ShowMessage("Error1");
	// empty = true;
	// }
 

  }

 /*The function apply_track is used to track the image for suitable lines. It
   *does this by first finding points above the highthreshold. When it finds
   *such a point it then finds surrounding point which are above the low threshold
   *and tracks along them, continually finding points above the low threshold. This
   *is done until the tracker explores all paths from the original point. It then
   *finds the next starting point and starts tracking again.
   */

  void apply_track(int **input,int **tracked,int width, int height,
			       int lowthresh,int highthresh) {
    int i,j;
    int **marked;
    d_w = width;
    d_h = height;


    //int [][] tracked = new int[d_w][d_h];

    marked= new int* [d_w];
    for (i=0;i<d_w;i++)
      marked[i]=new int [d_h];

    std::stack<int> to_track;

    //Initialise the marked array
    for( i = 0; i < d_w; i++){
      for( j = 0; j < d_h; j++){
	marked[i][j] = 0;
      }
    }

    //Now find all the starting points for the tracking
    for( i = 0; i < d_w; i++){
      for( j = 0; j < d_h; j++){
	//If the point is unmarked and above high threshold then track
	if((input[i][j] > highthresh) && (marked[i][j] == 0)){
	  track(input, marked, to_track, lowthresh, i, j);
	}
      }
    }

    //Now clear all the pixels in the input which are unmarked
    for( i = 0; i < d_w; i++){
      for( j = 0; j < d_h; j++){
	if(marked[i][j] == 0){
	  tracked[i][j] = 0;
	}
	else {
	  tracked[i][j] = input[i][j];
	}
      }
    }

    for(i=0;i<d_w;i++)
      delete(marked[i]);
    delete(marked);
 }



/**
   *Applies the canny edge detector to the input image
   *@param src_1d The source image as a pixel array
   *@param width width of the destination image in pixels
   *@param height height of the destination image in pixels
   *@param size The size of the kernel used in the smoothing
   *@param theta The gaussian smoothing standard deviation
   *@param lowthresh The low threshold for the tracking
   *@param highthresh The high threshold for the tracking
   *@param scale The amount of scaling to be applied to the image
   *@param offset The amount to be added to each result pixel
   *@return A pixel array containing edges in the image
   */

  //Tim's Canny Edge Detection Algorithm
  //Based on algorithm in Machine Vision (pg 169)
  /*a) assume the image is grey level
    b) gaussian smooth image
    c) work out gradient magnitude
    d) apply nonmaxima suppression
    e) threshold and detect edges
    */

  
  void CANNY::apply_canny() {

  int *dest_1d,**tmp_2d,**p_2d,**q_2d,**m_2d,**nms,**delta,**tracked,*tmp2_1d;
  double **theta_2d;
  int i,j;
  int result;
  //int tmp = 0;
  unsigned char **aux;

    d_w = height;
    d_h = width;

    for(i=0;i<height;i++){
    for(j=0;j<width;j++){
      im_lineas[i][j]=0xff000000;
    }
 }

    aux=new unsigned char* [d_w];
    dest_1d= new int[d_h*d_w];
    tmp2_1d=new int [d_h*d_w];
    tmp_2d=new int* [d_w];
    p_2d=new int* [d_w];
    q_2d=new int* [d_w];
    m_2d=new int* [d_w];
    theta_2d=new double* [d_w];
    nms=new int* [d_w];
    delta=new int* [d_w];
    tracked=new int* [d_w];
    for(i=0;i<d_w;i++){
      aux[i]=new unsigned char[d_h];
      tmp_2d[i]=new int[d_h];
      p_2d[i]=new int[d_h];
      q_2d[i]=new int[d_h];
      m_2d[i]=new int[d_h];
      theta_2d[i]=new double[d_h];
      nms[i]=new int[d_h];
      delta[i]=new int[d_h];
      tracked[i]=new int[d_h]; 
    } 

    //Smooth the initial image
		cout<<tmp_2d<<" "<<height<<" "<<width<<" "<<kernel_size<<" "<<theta<<endl;
    SmoothImage(im_entrada,tmp_2d,height, width, kernel_size, theta);

    //Mask off so that we work with values between 0 and 255
    for(i=0;i<height;i++){
    for(j=0;j<width;j++){
      tmp_2d[i][j]=tmp_2d[i][j]&0x000000ff;
    }
 }
    
    //Apply the gradient detection
    for(i = 0; i < (d_w-1); i++){
      for(j = 0; j < (d_h-1); j++){
	p_2d[i][j] = (tmp_2d[i][j+1]-tmp_2d[i][j]+
		      tmp_2d[i+1][j+1]-tmp_2d[i+1][j])/2;
	q_2d[i][j] = (tmp_2d[i][j]-tmp_2d[i+1][j]+
		      tmp_2d[i][j+1]-tmp_2d[i+1][j+1])/2;
	m_2d[i][j] = (int)sqrt(pow((double)p_2d[i][j],2)+
				    pow((double)q_2d[i][j],2));
        if((q_2d[i][j]==0)&&(p_2d[i][j]==0)){
             theta_2d[i][j]=0;
        }else{
	theta_2d[i][j] =atan2((double)(q_2d[i][j]),(double)(p_2d[i][j]));
        }
      }
    }

    //Resize image
    d_w--;
    d_h--;
    //Apply the nonmaxima suppression

    //First calculate which sector each line appears in

    for( i = 0; i < d_w; i++){
      for( j = 0; j < d_h; j++){
	delta[i][j] = sector(theta_2d[i][j]);
      }
    }

    //Then apply non maximal suppression
    for( i = 0; i < (d_w-1); i++){ nms[i][0] = 0; nms[i][d_h-1] = 0; }
    for( j = 0; j < (d_h-1); j++){ nms[0][j] = 0; nms[d_w-1][j] = 0; }
    for( i = 1; i < (d_w-1); i++){
      for( j = 1; j < (d_h-1); j++){
	nms[i][j] = suppress(m_2d, delta[i][j], i, j,lowthresh);
      }
    }

    //Resize again!
    d_w = d_w - 2;
    d_h = d_h - 2;

    //Track the image
     apply_track(nms,tracked, d_w, d_h, lowthresh, highthresh);

    //Calculate the output array
    for( i = 0; i < d_w; i++){
      for( j = 0; j < d_h; j++){
	result = tracked[i][j];
        //if(result!=0)ShowMessage("Aki");
	result = (int) (result * scale);
        if(result>0){
	result = result + offset;
        }
	if(result > 255){result = 255;}
	if(result < 0){result = 0;}
	im_lineas[i][j] =0xff000000|result;
      }
    }

    //Change the sizes back
    d_w = d_w + 3;
    d_h = d_h + 3;

    for(i=0;i<d_w;i++){
      delete(aux[i]);
      delete(tmp_2d[i]);
      delete(p_2d[i]);
      delete(q_2d[i]);
      delete(m_2d[i]);
      delete(theta_2d[i]);
      delete(nms[i]);
      delete(delta[i]);
      delete(tracked[i]); 
    } 
      delete(aux);
      delete(tmp_2d);
      delete(p_2d);
      delete(q_2d);
      delete(m_2d);
      delete(theta_2d);
      delete(nms);
      delete(delta);
      delete(tracked); 
      delete(dest_1d);
      delete(tmp2_1d);
}

void CANNY::Dilatacion(){
  int s;
  int i,j,k,l;

  for(k=1;k<height-1;k++){
   for(l=1;l<width-1;l++){
      s=im_lineas[k][l];
      for(i=0;i<2;i++){
         for(j=0;j<2;j++){
           if(s<im_lineas[k+i-1][l+j-1]) s=im_lineas[k+i-1][l+j-1];
         }
      }
      im_dilatada[k][l]=(unsigned char)s;
   }
}

}





