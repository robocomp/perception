
#include <stdio.h>
#include <math.h>
#include "Imagen.h"



/*********************

RGB_HSI		06/09/02

  Entradas: 
    R,G,B - campos red, green, blue
	H,S,I - campos hue, saturation, intensity
	Alto, Ancho - tamaño de la imagen (TAMBase x TAMBase)

  Salidas:
    Actualiza **H, **S y **I

  Pasa  una imagen definida por sus campos R, G y B a campos H, S e
  I.

*********************/

void RGB_HSI(IMAGEN_ENT *entrada)//unsigned char **im,double **H, double **S, double **I, int Alto, int Ancho)
{
  double min;
  double r, g, b,r1,g1,b1,h,s,v;
  double angle;
  double temp;
  register int i,j;

#if DEBUG
  FILE *outH, *outS, *outI;
  outH = fopen("H.raw", "wb");
  outS = fopen("S.raw", "wb");
  outI = fopen("I.raw", "wb");    
#endif

  for (i=0;i<entrada->alto; i++)
	for (j=0;j<entrada->ancho;j++)
	  {
	  r=((double) (entrada->im[i][j].R))/255;
	  g=((double) (entrada->im[i][j].G))/255;
	  b=((double) (entrada->im[i][j].B))/255;



       if(r==g && r==b){
         //I[i][j]=r;
         v=r;
         //H[i][j]=0;
         h=0;
         //S[i][j]=0;
         s=0;
       }
        else{
         if ((r>=g) && (r>=b))
	       v=r;//I[i][j]=r;
        else
		if ((g>=r) && (g>=b))
		  v=g;//I[i][j]=g;
		else
		  v=b;//I[i][j]=b;

       if (v==0){//I[i][j]==0) {
          s=0;//S[i][j]=0;
          h=0;//H[i][j]=0;

       }else {

             if ((r<=g) && (r<=b))
		min=r;
             else
		if ((g<=r) && (g<=b))
		  min=g;
		else
 		  min=b;

             s=(v-min)/v;//S[i][j]=(I[i][j]-min)/I[i][j];
             r1=(v-r)/(v-min);//r1=(I[i][j]-r)/(I[i][j]-min);
             g1=(v-g)/(v-min);//g1=(I[i][j]-g)/(I[i][j]-min);
             b1=(v-b)/(v-min);//b1=(I[i][j]-b)/(I[i][j]-min);

             if(r==v){//r==I[i][j]){
                h=(g1-b1)*(3.1415/3);//H[i][j]=(g1-b1)*(3.1415/3);
                if ( h<3.1415/2)//H[i][j]<3.1415/2)
                          h=(2*3.1415)-h;//H[i][j]=(2*3.1415)-H[i][j];

             }
             if(g==v){//I[i][j]){
                h=(2+r1-b1)*(3.1415/3);//H[i][j]=(2+r1-b1)*(3.1415/3);

             }
             if(b==v){//b==I[i][j]){
                h=(4+g1-r1)*(3.1415/3);//H[i][j]=(4+g1-r1)*(3.1415/3);

             }

       }
     }
        v=v*255;// I[i][j]=I[i][j]*255;
        s=s*255;//S[i][j]=S[i][j]*255;
        h=h*180/3.1415;//H[i][j]=H[i][j]*180/3.1415;
       if(h>360)//H[i][j]>360)
          h=h-360;//H[i][j]=H[i][j]-360;
       if(h<0)//H[i][j]<0)
          h=360+h;//H[i][j]=360+H[i][j];
       h=h*255/360;//H[i][j]=H[i][j]*255/360;
       // S[i][j]=310*(1-exp(-1*(S[i][j]/150)));

       entrada->im[i][j].C=(unsigned char)h;
       entrada->im[i][j].S=(unsigned char)s;
       entrada->im[i][j].I=(unsigned char)v;

#if DEBUG
  putc((unsigned char)((H[i][j]*255)/360), outH);
  putc((unsigned char)S[i][j], outS);
  putc((unsigned char)I[i][j], outI);
#endif
	  }



#if DEBUG
  fcloseall();
#endif
}

void HSI_RGB(IMAGEN_SEG *segmentada){

  double H,S,I,S2;
  double h,P,Q,T;
  double temp;
  int i,j;


   for (i=0;i<segmentada->alto; i++){
	for (j=0;j<segmentada->ancho;j++){
          //R[i][j]=G[i][j]=B[i][j]=0;
          segmentada->im[i][j].R=0;
          segmentada->im[i][j].G=0;
          segmentada->im[i][j].B=0;

        }
   }

  for (i=0;i<segmentada->alto; i++){
	for (j=0;j<segmentada->ancho;j++)
	  {
            //I=I1[i][j]/255;
            I=((double)(segmentada->im[i][j].I))/255;
	    // S2=(-150)*log(1-(S1[i][j]/310));
	    //S2=S1[i][j];
            S2=segmentada->im[i][j].S;
            S=S2/255;
            //H=H1[i][j]*360/255;
            H=((double)(segmentada->im[i][j].C))*360/255;
            if(H<0)
                H=360+H;
            H=H*3.1415/180;

          if(S==0){

            segmentada->im[i][j].R=(unsigned char)(I*255);
            segmentada->im[i][j].G=(unsigned char)(I*255);
            segmentada->im[i][j].B=(unsigned char)(I*255);

          } else{

            if (I!=0){

              H=H*3/3.1415;
              h=floor(H);
              if(h==6)
                h=5;
              P=I*(1-S);
              Q=I*(1-(S*(H-h)));
              T=I*(1-(S*(1-H+h)));

              switch ((int)h){

                 case 0:
                    segmentada->im[i][j].R=(unsigned char)(I*255);
                    segmentada->im[i][j].G=(unsigned char)(T*255);
                    segmentada->im[i][j].B=(unsigned char)(P*255);
                    break;
                 case 1:
                    segmentada->im[i][j].R=(unsigned char)(Q*255);
                    segmentada->im[i][j].G=(unsigned char)(I*255);
                    segmentada->im[i][j].B=(unsigned char)(P*255);
                    break;
                 case 2:
                    segmentada->im[i][j].R=(unsigned char)(P*255);
                    segmentada->im[i][j].G=(unsigned char)(I*255);
                    segmentada->im[i][j].B=(unsigned char)(T*255);
                    break;
                 case 3:
                    segmentada->im[i][j].R=(unsigned char)(P*255);
                    segmentada->im[i][j].G=(unsigned char)(Q*255);
                    segmentada->im[i][j].B=(unsigned char)(I*255);
                    break;
                 case 4:
                    segmentada->im[i][j].R=(unsigned char)(T*255);
                    segmentada->im[i][j].G=(unsigned char)(P*255);
                    segmentada->im[i][j].B=(unsigned char)(I*255);
                    break;
                 case 5:
                    segmentada->im[i][j].R=(unsigned char)(I*255);
                    segmentada->im[i][j].G=(unsigned char)(P*255);
                    segmentada->im[i][j].B=(unsigned char)(Q*255);
                    break;

              }

            }
          }


 }
}

}


