#include "BIP.h"
#include <math.h>
#include <stdio.h>
#include <algorithm>
#include <iostream>
#include <float.h>


BIP::BIP(int _altoim, int _anchoim, int _nivelesI, double umbral){

register int i,j,k;
int lado;

nivelesC=30;

altoim=_altoim;
anchoim=_anchoim;
if (altoim>anchoim)
        lado=altoim;
else
        lado=anchoim;

double seis = log10((double)lado)/log10(2.0);
niveles=static_cast<int>(seis+1.0);

nivelesI=_nivelesI;

TamNivelesAlto=new int [niveles];
TamNivelesAncho=new int [niveles];
TamNivelesAlto[0]=altoim;
TamNivelesAncho[0]=anchoim;
for (i=1;i<niveles;i++)
        TamNivelesAlto[i]=TamNivelesAlto[i-1]/2;
TamNivelesAncho[0]=anchoim;
for (i=1;i<niveles;i++)
        TamNivelesAncho[i]=TamNivelesAncho[i-1]/2;


NodosV=new std::vector<NV> [nivelesI];

NodosC=new std::vector<NV> [nivelesC];

pir=new PIR**[niveles];
for(i=0;i<niveles;i++){
        pir[i]=new PIR*[TamNivelesAlto[i]];
        for(j=0;j<TamNivelesAlto[i];j++)
                pir[i][j]=new PIR[TamNivelesAncho[i]];
}



   for(i=0;i<altoim;i++){
        for(j=0;j<anchoim;j++){
                pir[0][i][j].X=-1;
                pir[0][i][j].Y=-1;
                pir[0][i][j].Homog=1;
                pir[0][i][j].Area=1;
                pir[0][i][j].Clase=-1;

        }
}
for(k=1;k<niveles;k++)
        for(i=0;i<TamNivelesAlto[k];i++)
                for(j=0;j<TamNivelesAncho[k];j++){
                        pir[k][i][j].X=-1;
                        pir[k][i][j].Y=-1;
                        pir[k][i][j].Homog=-1;
                        pir[k][i][j].Area=0;
                        pir[k][i][j].Clase=-1;
                }



umbralColor=new double[9];
 umbralColor[0]=umbral;//50;
 umbralColor[1]=umbral;//50;
 umbralColor[2]=umbral;//40;
 umbralColor[3]=umbral;//40;
 umbralColor[4]=umbral;//40;
 umbralColor[5]=umbral;//40;
 umbralColor[6]=umbral;//40;
 umbralColor[7]=umbral;//40;
 umbralColor[8]=umbral;//40;

}

BIP::~BIP(){

register int i,j;

for(i=0;i<niveles;i++){
  for(j=0;j<TamNivelesAlto[i];j++){
    for(int k=0;k<TamNivelesAncho[i];k++){
      pir[i][j][k].contactosV.clear();
      pir[i][j][k].contactosR.clear();
    }
  }
 }

for(i=0;i<niveles;i++){
  for(j=0;j<TamNivelesAlto[i];j++){
    delete[]pir[i][j];
  }
  delete[]pir[i];
  }

  delete[]pir;

delete[]umbralColor;
delete[]TamNivelesAlto;
delete[]TamNivelesAncho;

for(i=0;i<nivelesI;i++){
   for(j=0;j<(int)NodosV[i].size();j++){
   NodosV[i][j].contactos.clear();
   NodosV[i][j].contactosR.clear();
   NodosV[i][j].contraste.clear();
   NodosV[i][j].num_pixel_contacto.clear();
   NodosV[i][j].num_pixel_contacto_Canny.clear();
   NodosV[i][j].hijos.clear();
 }
   NodosV[i].clear();
 }
delete[]NodosV;

for(i=0;i<nivelesC;i++){
   for(j=0;j<(int)NodosC[i].size();j++){
   NodosC[i][j].contactos.clear();
   NodosC[i][j].contactosR.clear();
   NodosC[i][j].contraste.clear();
   NodosC[i][j].num_pixel_contacto.clear();
   NodosC[i][j].num_pixel_contacto_Canny.clear();
   NodosC[i][j].hijos.clear();
   }
   NodosC[i].clear();
 }

  delete[]NodosC;

 padres.clear();

 delete[]cajas;
 delete[]cajasmin;


}
/***********************************************************
GENERAR PARTE REGULAR: Genera la estructura 4 a 1 de la parte regular de la BIP
**********************************************************************/
int BIP::GenerarParteRegular(int nivel){

register int i,j,i1,j1,i2,j2,k;
double teta,H;
PUNTO p1,p2,p3,p4,medio;
int generado=-1;

k=nivel+1;

for(i=0;i<TamNivelesAlto[k];i++){
        i1=i*2;
        i2=i1+1;
        for(j=0;j<TamNivelesAncho[k];j++){
                j1=j*2;
                j2=j1+1;
                if(pir[nivel][i1][j1].Homog==1 && pir[nivel][i2][j1].Homog==1
                   &&pir[nivel][i1][j2].Homog==1 &&pir[nivel][i2][j2].Homog==1){

                        if(DisColor(pir[nivel][i1][j1],pir[nivel][i2][j1])<umbralColor[nivel]&&
                          DisColor(pir[nivel][i1][j1],pir[nivel][i1][j2])<umbralColor[nivel]&&
                           DisColor(pir[nivel][i1][j1],pir[nivel][i2][j2])<umbralColor[nivel]){

                                generado=1;
				pir[k][i][j].Homog=1;

				pir[k][i][j].Area=pir[nivel][i1][j1].Area+pir[nivel][i1][j2].Area+pir[nivel][i2][j1].Area+pir[nivel][i2][j2].Area;

				pir[k][i][j].I=((pir[nivel][i1][j1].I*pir[nivel][i1][j1].Area)+(pir[nivel][i1][j2].I*pir[nivel][i1][j2].Area)+
                                        (pir[nivel][i2][j1].I*pir[nivel][i2][j1].Area)+(pir[nivel][i2][j2].I*pir[nivel][i2][j2].Area))/pir[k][i][j].Area;


                                teta=pir[nivel][i1][j1].C*2*3.1415/255;
                                p1.i=(pir[nivel][i1][j1].S*cos(teta));
                                p1.j=(pir[nivel][i1][j1].S*sin(teta));

                                teta=pir[nivel][i1][j2].C*2*3.1415/255;
                                p2.i=(pir[nivel][i1][j2].S*cos(teta));
                                p2.j=(pir[nivel][i1][j2].S*sin(teta));

                                teta=pir[nivel][i2][j1].C*2*3.1415/255;
                                p3.i=(pir[nivel][i2][j1].S*cos(teta));
                                p3.j=(pir[nivel][i2][j1].S*sin(teta));

                                teta=pir[nivel][i2][j2].C*2*3.1415/255;
                                p4.i=(pir[nivel][i2][j2].S*cos(teta));
                                p4.j=(pir[nivel][i2][j2].S*sin(teta));



                                medio.i=(p1.i+p2.i+p3.i+p4.i)/4;
                                medio.j=(p1.j+p2.j+p3.j+p4.j)/4;

                                if(medio.j==0 && medio.i==0) H=0;
                                else H=(atan2(medio.j,medio.i))*180/3.1415;

                                if(H<0) H=360+H;
                                H=H*255/360;

                                pir[k][i][j].C=(unsigned char)H;

                                pir[k][i][j].S=(unsigned char)sqrt((medio.i*medio.i)+(medio.j*medio.j));

				pir[k][i][j].Cor=pir[k][i][j].C;
				pir[k][i][j].Sor=pir[k][i][j].S;
				pir[k][i][j].Ior=pir[k][i][j].I;

             //Actualizo la posición del nodo padre en los 4 nodos del nivel
             //inferior

                                pir[nivel][i1][j1].Y=j;
		                pir[nivel][i1][j2].Y=j;
		                pir[nivel][i2][j1].Y=j;
                                pir[nivel][i2][j2].Y=j;
                                pir[nivel][i1][j1].X=i;
		                pir[nivel][i1][j2].X=i;
		                pir[nivel][i2][j1].X=i;
                                pir[nivel][i2][j2].X=i;

                        }

                   }
        }
}
return generado;
}

/**************************************************************************++++
CLASIFICAR: Asigna valores de clase a los nodos de la estructura sin padre y
distribuye estos valores entre sus hijos. Además almacena los colores de estos
padres raíz para generar la imagen segmentada.
************************************************************************************/
void BIP::Clasificar(){
register int i,j,k;
int homog,x,y;
ContClase=0;
Padre papa;

for(k=nivelesI-1;k>=niveles;k--){
 for(i=0;i<(int)NodosV[k].size();i++){
   if(NodosV[k][i].padre==-1){

                NodosV[k][i].Clase=ContClase;
                ContClase++;
                papa.Area=NodosV[k][i].Area;
                papa.posi=-1;
                papa.posj=i;
                papa.Niv=k;
                papa.NodoV=1;
                papa.C=NodosV[k][i].C;
                papa.S=NodosV[k][i].S;
                papa.I=NodosV[k][i].I;
                padres.push_back(papa);

            } else if (NodosV[k][i].padre!=-1){

                NodosV[k][i].Clase=NodosV[k+1][NodosV[k][i].padre].Clase;
            }

    }
    }
  for (k=niveles-1;k>=0;k--)
    {
    for(i=0;i<(int)NodosV[k].size();i++){
          if (NodosV[k][i].padre!=-1){

                NodosV[k][i].Clase=NodosV[k+1][NodosV[k][i].padre].Clase;
            }
          else if(NodosV[k][i].padre==-1){

                NodosV[k][i].Clase=ContClase;
                ContClase++;
                papa.Area=NodosV[k][i].Area;
                papa.posi=-1;
                papa.posj=i;
                papa.Niv=k;
                papa.NodoV=-1;
                papa.C=NodosV[k][i].C;
                papa.S=NodosV[k][i].S;
                papa.I=NodosV[k][i].I;
                padres.push_back(papa);
            }
    }

    for (i=0;i<TamNivelesAlto[k];i++){
       for(j=0;j<TamNivelesAncho[k];j++)
	  {
          homog=pir[k][i][j].Homog;
          x=pir[k][i][j].X;
          y=pir[k][i][j].Y;

      if ((homog==1)&& x!=-1 && y!=-1)
		  {
		  /* Padre natural */

            pir[k][i][j].Clase=pir[k+1][x][y].Clase;


		  }
      else if (homog==1 && x==-1 && y!=-1){
                  pir[k][i][j].Clase=NodosV[k+1][y].Clase;

	  /* CASO 1: Clase nueva */
      }else if ((homog==1 && x==-1 && y==-1))
		{
                pir[k][i][j].Clase=ContClase;
                papa.Area=pir[k][i][j].Area;
                papa.posi=i;
                papa.posj=j;
                papa.Niv=k;
                papa.NodoV=-1;
                papa.C=pir[k][i][j].C;
                papa.S=pir[k][i][j].S;
                papa.I=pir[k][i][j].I;
                padres.push_back(papa);
	 	ContClase++;
        }

    }//for
    }//for
  }//for
}

/************************************************************
LLevarColoresArriba: Para propagar los colores de las regiones
generadas en la segmentación a traves de la estructura.

*************************************************************/

void BIP::LLevarColoresArriba(){

  register int i,j;

    for(i=0;i<TamNivelesAlto[0];i++)
      for(j=0;j<TamNivelesAncho[0];j++){
        pir[0][i][j].C=padres[pir[0][i][j].Clase].C;
        pir[0][i][j].S=padres[pir[0][i][j].Clase].S;
	pir[0][i][j].I=padres[pir[0][i][j].Clase].I;

      }


}
/************************************************************************************
CALCULAR ADYACENCIA REGULAR: Si dos nodos regulares están en contacto en un nivel
sus padres estarán en contacto en el nivel superior.
***************************************************************************************/
void CalcularAdyacenciaRegular(std::vector<NV> &nodos, PIR ***pir, int tamAlto, int tamAncho, int nivel){

register int i,j;
PUNTO coor,coor1;


for(i=0;i<tamAlto;i++){
   for(j=0;j<tamAncho;j++){

   //Si dos nodos regulares están en contacto y sus padres son uno un nodo virtual y el otro un nodo regular
  //entonces estos dos nodos padres están en contacto en el nivel superior
  if (pir[nivel][i][j].X!=-1 && pir[nivel][i][j].Y!=-1){ // nodo 1 padre regular nodo2 padre regular o virtual
    coor.i= pir[nivel][i][j].X;
    coor.j= pir[nivel][i][j].Y;

    if((j-1!=-1)&&pir[nivel][i][j-1].Y!=-1 &&pir[nivel][i][j-1].X==-1){
        //Para comprobar si el contacto estaba ya incluido
        if(std::find(pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosV.begin(),pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosV.end(),
          pir[nivel][i][j-1].Y)==pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosV.end()){
                pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosV.push_back(pir[nivel][i][j-1].Y);
          }
       if(std::find_if(nodos[pir[nivel][i][j-1].Y].contactosR.begin(),nodos[pir[nivel][i][j-1].Y].contactosR.end(),
          CmpContactos(coor))==nodos[pir[nivel][i][j-1].Y].contactosR.end()){
                nodos[pir[nivel][i][j-1].Y].contactosR.push_back(coor);
          }
      } else if (((j-1!=-1)&&pir[nivel][i][j-1].Y!=-1 &&pir[nivel][i][j-1].X!=-1)&&
                ((pir[nivel][i][j-1].X!=pir[nivel][i][j].X||pir[nivel][i][j-1].Y!=pir[nivel][i][j].Y)&&
                (pir[nivel][i][j-1].X!=pir[nivel][i][j].X-1||pir[nivel][i][j-1].Y!=pir[nivel][i][j].Y-1)&&
                (pir[nivel][i][j-1].X!=pir[nivel][i][j].X-1||pir[nivel][i][j-1].Y!=pir[nivel][i][j].Y)&&
                (pir[nivel][i][j-1].X!=pir[nivel][i][j].X-1||pir[nivel][i][j-1].Y!=pir[nivel][i][j].Y+1)&&
                (pir[nivel][i][j-1].X!=pir[nivel][i][j].X||pir[nivel][i][j-1].Y!=pir[nivel][i][j].Y-1)&&
                (pir[nivel][i][j-1].X!=pir[nivel][i][j].X||pir[nivel][i][j-1].Y!=pir[nivel][i][j].Y+1)&&
                (pir[nivel][i][j-1].X!=pir[nivel][i][j].X+1||pir[nivel][i][j-1].Y!=pir[nivel][i][j].Y-1)&&
                (pir[nivel][i][j-1].X!=pir[nivel][i][j].X+1||pir[nivel][i][j-1].Y!=pir[nivel][i][j].Y)&&
                (pir[nivel][i][j-1].X!=pir[nivel][i][j].X+1||pir[nivel][i][j-1].Y!=pir[nivel][i][j].Y+1))){

                coor1.i=pir[nivel][i][j-1].X;
                coor1.j=pir[nivel][i][j-1].Y;

                 if(std::find_if(pir[nivel+1][pir[nivel][i][j-1].X][pir[nivel][i][j-1].Y].contactosR.begin(),pir[nivel+1][pir[nivel][i][j-1].X][pir[nivel][i][j-1].Y].contactosR.end(),
                 CmpContactos(coor))==pir[nivel+1][pir[nivel][i][j-1].X][pir[nivel][i][j-1].Y].contactosR.end()){
                 pir[nivel+1][pir[nivel][i][j-1].X][pir[nivel][i][j-1].Y].contactosR.push_back(coor);
                 }
                 if(std::find_if(pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosR.begin(),pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosR.end(),
                 CmpContactos(coor1))==pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosR.end()){
                 pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosR.push_back(coor1);
                }
      }

      if((j-1!=-1)&&(i-1!=-1)&&pir[nivel][i-1][j-1].Y!=-1 &&pir[nivel][i-1][j-1].X==-1){
        //Para comprobar si el contacto estaba ya incluido
        if(std::find(pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosV.begin(),pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosV.end(),
          pir[nivel][i-1][j-1].Y)==pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosV.end()){
                pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosV.push_back(pir[nivel][i-1][j-1].Y);
          }
       if(std::find_if(nodos[pir[nivel][i-1][j-1].Y].contactosR.begin(),nodos[pir[nivel][i-1][j-1].Y].contactosR.end(),
          CmpContactos(coor))==nodos[pir[nivel][i-1][j-1].Y].contactosR.end()){
                nodos[pir[nivel][i-1][j-1].Y].contactosR.push_back(coor);
          }
      } else if (((j-1!=-1)&&(i-1!=-1)&&pir[nivel][i-1][j-1].Y!=-1 &&pir[nivel][i-1][j-1].X!=-1)&&
                ((pir[nivel][i-1][j-1].X!=pir[nivel][i][j].X||pir[nivel][i-1][j-1].Y!=pir[nivel][i][j].Y)&&
                (pir[nivel][i-1][j-1].X!=pir[nivel][i][j].X-1||pir[nivel][i-1][j-1].Y!=pir[nivel][i][j].Y-1)&&
                (pir[nivel][i-1][j-1].X!=pir[nivel][i][j].X-1||pir[nivel][i-1][j-1].Y!=pir[nivel][i][j].Y)&&
                (pir[nivel][i-1][j-1].X!=pir[nivel][i][j].X-1||pir[nivel][i-1][j-1].Y!=pir[nivel][i][j].Y+1)&&
                (pir[nivel][i-1][j-1].X!=pir[nivel][i][j].X||pir[nivel][i-1][j-1].Y!=pir[nivel][i][j].Y-1)&&
                (pir[nivel][i-1][j-1].X!=pir[nivel][i][j].X||pir[nivel][i-1][j-1].Y!=pir[nivel][i][j].Y+1)&&
                (pir[nivel][i-1][j-1].X!=pir[nivel][i][j].X+1||pir[nivel][i-1][j-1].Y!=pir[nivel][i][j].Y-1)&&
                (pir[nivel][i-1][j-1].X!=pir[nivel][i][j].X+1||pir[nivel][i-1][j-1].Y!=pir[nivel][i][j].Y)&&
                (pir[nivel][i-1][j-1].X!=pir[nivel][i][j].X+1||pir[nivel][i-1][j-1].Y!=pir[nivel][i][j].Y+1))){

                coor1.i=pir[nivel][i-1][j-1].X;
                coor1.j=pir[nivel][i-1][j-1].Y;

                 if(std::find_if(pir[nivel+1][pir[nivel][i-1][j-1].X][pir[nivel][i-1][j-1].Y].contactosR.begin(),pir[nivel+1][pir[nivel][i-1][j-1].X][pir[nivel][i-1][j-1].Y].contactosR.end(),
                 CmpContactos(coor))==pir[nivel+1][pir[nivel][i-1][j-1].X][pir[nivel][i-1][j-1].Y].contactosR.end()){
                 pir[nivel+1][pir[nivel][i-1][j-1].X][pir[nivel][i-1][j-1].Y].contactosR.push_back(coor);
                 }
                 if(std::find_if(pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosR.begin(),pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosR.end(),
                 CmpContactos(coor1))==pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosR.end()){
                 pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosR.push_back(coor1);
                }
      }


      if((i-1!=-1)&&pir[nivel][i-1][j].Y!=-1 &&pir[nivel][i-1][j].X==-1){
        //Para comprobar si el contacto estaba ya incluido
        if(std::find(pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosV.begin(),pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosV.end(),
          pir[nivel][i-1][j].Y)==pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosV.end()){
                pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosV.push_back(pir[nivel][i-1][j].Y);
          }
       if(std::find_if(nodos[pir[nivel][i-1][j].Y].contactosR.begin(),nodos[pir[nivel][i-1][j].Y].contactosR.end(),
          CmpContactos(coor))==nodos[pir[nivel][i-1][j].Y].contactosR.end()){
                nodos[pir[nivel][i-1][j].Y].contactosR.push_back(coor);
          }
      } else if (((i-1!=-1)&&pir[nivel][i-1][j].Y!=-1 &&pir[nivel][i-1][j].X!=-1)&&
                ((pir[nivel][i-1][j].X!=pir[nivel][i][j].X||pir[nivel][i-1][j].Y!=pir[nivel][i][j].Y)&&
                (pir[nivel][i-1][j].X!=pir[nivel][i][j].X-1||pir[nivel][i-1][j].Y!=pir[nivel][i][j].Y-1)&&
                (pir[nivel][i-1][j].X!=pir[nivel][i][j].X-1||pir[nivel][i-1][j].Y!=pir[nivel][i][j].Y)&&
                (pir[nivel][i-1][j].X!=pir[nivel][i][j].X-1||pir[nivel][i-1][j].Y!=pir[nivel][i][j].Y+1)&&
                (pir[nivel][i-1][j].X!=pir[nivel][i][j].X||pir[nivel][i-1][j].Y!=pir[nivel][i][j].Y-1)&&
                (pir[nivel][i-1][j].X!=pir[nivel][i][j].X||pir[nivel][i-1][j].Y!=pir[nivel][i][j].Y+1)&&
                (pir[nivel][i-1][j].X!=pir[nivel][i][j].X+1||pir[nivel][i-1][j].Y!=pir[nivel][i][j].Y-1)&&
                (pir[nivel][i-1][j].X!=pir[nivel][i][j].X+1||pir[nivel][i-1][j].Y!=pir[nivel][i][j].Y)&&
                (pir[nivel][i-1][j].X!=pir[nivel][i][j].X+1||pir[nivel][i-1][j].Y!=pir[nivel][i][j].Y+1))){

                coor1.i=pir[nivel][i-1][j].X;
                coor1.j=pir[nivel][i-1][j].Y;

                 if(std::find_if(pir[nivel+1][pir[nivel][i-1][j].X][pir[nivel][i-1][j].Y].contactosR.begin(),pir[nivel+1][pir[nivel][i-1][j].X][pir[nivel][i-1][j].Y].contactosR.end(),
                 CmpContactos(coor))==pir[nivel+1][pir[nivel][i-1][j].X][pir[nivel][i-1][j].Y].contactosR.end()){
                 pir[nivel+1][pir[nivel][i-1][j].X][pir[nivel][i-1][j].Y].contactosR.push_back(coor);
                 }
                 if(std::find_if(pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosR.begin(),pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosR.end(),
                 CmpContactos(coor1))==pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosR.end()){
                 pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosR.push_back(coor1);
                }
      }

      if((j+1!=tamAncho)&&(i-1!=-1)&&pir[nivel][i-1][j+1].Y!=-1 &&pir[nivel][i-1][j+1].X==-1){
        //Para comprobar si el contacto estaba ya incluido
        if(std::find(pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosV.begin(),pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosV.end(),
          pir[nivel][i-1][j+1].Y)==pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosV.end()){
                pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosV.push_back(pir[nivel][i-1][j+1].Y);
          }
       if(std::find_if(nodos[pir[nivel][i-1][j+1].Y].contactosR.begin(),nodos[pir[nivel][i-1][j+1].Y].contactosR.end(),
          CmpContactos(coor))==nodos[pir[nivel][i-1][j+1].Y].contactosR.end()){
                nodos[pir[nivel][i-1][j+1].Y].contactosR.push_back(coor);
          }
      }else if (((j+1!=tamAncho)&&(i-1!=-1)&&pir[nivel][i-1][j+1].Y!=-1 &&pir[nivel][i-1][j+1].X!=-1)&&
                ((pir[nivel][i-1][j+1].X!=pir[nivel][i][j].X||pir[nivel][i-1][j+1].Y!=pir[nivel][i][j].Y)&&
                (pir[nivel][i-1][j+1].X!=pir[nivel][i][j].X-1||pir[nivel][i-1][j+1].Y!=pir[nivel][i][j].Y-1)&&
                (pir[nivel][i-1][j+1].X!=pir[nivel][i][j].X-1||pir[nivel][i-1][j+1].Y!=pir[nivel][i][j].Y)&&
                (pir[nivel][i-1][j+1].X!=pir[nivel][i][j].X-1||pir[nivel][i-1][j+1].Y!=pir[nivel][i][j].Y+1)&&
                (pir[nivel][i-1][j+1].X!=pir[nivel][i][j].X||pir[nivel][i-1][j+1].Y!=pir[nivel][i][j].Y-1)&&
                (pir[nivel][i-1][j+1].X!=pir[nivel][i][j].X||pir[nivel][i-1][j+1].Y!=pir[nivel][i][j].Y+1)&&
                (pir[nivel][i-1][j+1].X!=pir[nivel][i][j].X+1||pir[nivel][i-1][j+1].Y!=pir[nivel][i][j].Y-1)&&
                (pir[nivel][i-1][j+1].X!=pir[nivel][i][j].X+1||pir[nivel][i-1][j+1].Y!=pir[nivel][i][j].Y)&&
                (pir[nivel][i-1][j+1].X!=pir[nivel][i][j].X+1||pir[nivel][i-1][j+1].Y!=pir[nivel][i][j].Y+1))){

                coor1.i=pir[nivel][i-1][j+1].X;
                coor1.j=pir[nivel][i-1][j+1].Y;

                 if(std::find_if(pir[nivel+1][pir[nivel][i-1][j+1].X][pir[nivel][i-1][j+1].Y].contactosR.begin(),pir[nivel+1][pir[nivel][i-1][j+1].X][pir[nivel][i-1][j+1].Y].contactosR.end(),
                 CmpContactos(coor))==pir[nivel+1][pir[nivel][i-1][j+1].X][pir[nivel][i-1][j+1].Y].contactosR.end()){
                 pir[nivel+1][pir[nivel][i-1][j+1].X][pir[nivel][i-1][j+1].Y].contactosR.push_back(coor);
                 }
                 if(std::find_if(pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosR.begin(),pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosR.end(),
                 CmpContactos(coor1))==pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosR.end()){
                 pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].contactosR.push_back(coor1);
                }

    }
   }
   else if(pir[nivel][i][j].X==-1&&pir[nivel][i][j].Y!=-1){ //nodo 1 padre virtual  nodo2 padre regular o virtual
        if((j-1!=-1)&&pir[nivel][i][j-1].Y!=-1 &&pir[nivel][i][j-1].X==-1 && pir[nivel][i][j].Y!=pir[nivel][i][j-1].Y){
        //Para comprobar si el contacto estaba ya incluido
        if(std::find(nodos[pir[nivel][i][j].Y].contactos.begin(),nodos[pir[nivel][i][j].Y].contactos.end(),
          pir[nivel][i][j-1].Y)==nodos[pir[nivel][i][j].Y].contactos.end()){
                nodos[pir[nivel][i][j].Y].contactos.push_back(pir[nivel][i][j-1].Y);
          }
       if(std::find(nodos[pir[nivel][i][j-1].Y].contactos.begin(),nodos[pir[nivel][i][j-1].Y].contactos.end(),
          pir[nivel][i][j].Y)==nodos[pir[nivel][i][j-1].Y].contactos.end()){
                nodos[pir[nivel][i][j-1].Y].contactos.push_back(pir[nivel][i][j].Y);
          }
      } else if((j-1!=-1)&&pir[nivel][i][j-1].Y!=-1 &&pir[nivel][i][j-1].X!=-1){
        coor.i=pir[nivel][i][j-1].X;
        coor.j=pir[nivel][i][j-1].Y;
        if(std::find(pir[nivel+1][pir[nivel][i][j-1].X][pir[nivel][i][j-1].Y].contactosV.begin(),pir[nivel+1][pir[nivel][i][j-1].X][pir[nivel][i][j-1].Y].contactosV.end(),
          pir[nivel][i][j].Y)==pir[nivel+1][pir[nivel][i][j-1].X][pir[nivel][i][j-1].Y].contactosV.end()){
                pir[nivel+1][pir[nivel][i][j-1].X][pir[nivel][i][j-1].Y].contactosV.push_back(pir[nivel][i][j].Y);
          }
       if(std::find_if(nodos[pir[nivel][i][j].Y].contactosR.begin(),nodos[pir[nivel][i][j].Y].contactosR.end(),
          CmpContactos(coor))==nodos[pir[nivel][i][j].Y].contactosR.end()){
                nodos[pir[nivel][i][j].Y].contactosR.push_back(coor);
          }
      }

        if((j-1!=-1)&&(i-1!=-1)&&
        pir[nivel][i-1][j-1].Y!=-1 &&pir[nivel][i-1][j-1].X==-1 && pir[nivel][i][j].Y!=pir[nivel][i-1][j-1].Y){
           if(std::find(nodos[pir[nivel][i][j].Y].contactos.begin(),nodos[pir[nivel][i][j].Y].contactos.end(),
                pir[nivel][i-1][j-1].Y)==nodos[pir[nivel][i][j].Y].contactos.end()){
                nodos[pir[nivel][i][j].Y].contactos.push_back(pir[nivel][i-1][j-1].Y);
          }
         if(std::find(nodos[pir[nivel][i-1][j-1].Y].contactos.begin(),nodos[pir[nivel][i-1][j-1].Y].contactos.end(),
          pir[nivel][i][j].Y)==nodos[pir[nivel][i-1][j-1].Y].contactos.end()){
                nodos[pir[nivel][i-1][j-1].Y].contactos.push_back(pir[nivel][i][j].Y);
          }
      } else if((j-1!=-1)&&(i-1!=-1)&&pir[nivel][i-1][j-1].Y!=-1 &&pir[nivel][i-1][j-1].X!=-1){
        coor.i=pir[nivel][i-1][j-1].X;
        coor.j=pir[nivel][i-1][j-1].Y;
        if(std::find(pir[nivel+1][pir[nivel][i-1][j-1].X][pir[nivel][i-1][j-1].Y].contactosV.begin(),pir[nivel+1][pir[nivel][i-1][j-1].X][pir[nivel][i-1][j-1].Y].contactosV.end(),
          pir[nivel][i][j].Y)==pir[nivel+1][pir[nivel][i-1][j-1].X][pir[nivel][i-1][j-1].Y].contactosV.end()){
                pir[nivel+1][pir[nivel][i-1][j-1].X][pir[nivel][i-1][j-1].Y].contactosV.push_back(pir[nivel][i][j].Y);
          }
       if(std::find_if(nodos[pir[nivel][i][j].Y].contactosR.begin(),nodos[pir[nivel][i][j].Y].contactosR.end(),
          CmpContactos(coor))==nodos[pir[nivel][i][j].Y].contactosR.end()){
                nodos[pir[nivel][i][j].Y].contactosR.push_back(coor);
          }
      }


      if((i-1!=-1)&&pir[nivel][i-1][j].Y!=-1 &&pir[nivel][i-1][j].X==-1 && pir[nivel][i][j].Y!=pir[nivel][i-1][j].Y){
        if(std::find(nodos[pir[nivel][i][j].Y].contactos.begin(),nodos[pir[nivel][i][j].Y].contactos.end(),
          pir[nivel][i-1][j].Y)==nodos[pir[nivel][i][j].Y].contactos.end()){
                nodos[pir[nivel][i][j].Y].contactos.push_back(pir[nivel][i-1][j].Y);
           }
          if(std::find(nodos[pir[nivel][i-1][j].Y].contactos.begin(),nodos[pir[nivel][i-1][j].Y].contactos.end(),
          pir[nivel][i][j].Y)==nodos[pir[nivel][i-1][j].Y].contactos.end()){
                nodos[pir[nivel][i-1][j].Y].contactos.push_back(pir[nivel][i][j].Y);
          }
      } else if((i-1!=-1)&&pir[nivel][i-1][j].Y!=-1 &&pir[nivel][i-1][j].X!=-1){
        coor.i=pir[nivel][i-1][j].X;
        coor.j=pir[nivel][i-1][j].Y;
        if(std::find(pir[nivel+1][pir[nivel][i-1][j].X][pir[nivel][i-1][j].Y].contactosV.begin(),pir[nivel+1][pir[nivel][i-1][j].X][pir[nivel][i-1][j].Y].contactosV.end(),
          pir[nivel][i][j].Y)==pir[nivel+1][pir[nivel][i-1][j].X][pir[nivel][i-1][j].Y].contactosV.end()){
                pir[nivel+1][pir[nivel][i-1][j].X][pir[nivel][i-1][j].Y].contactosV.push_back(pir[nivel][i][j].Y);
          }
       if(std::find_if(nodos[pir[nivel][i][j].Y].contactosR.begin(),nodos[pir[nivel][i][j].Y].contactosR.end(),
          CmpContactos(coor))==nodos[pir[nivel][i][j].Y].contactosR.end()){
                nodos[pir[nivel][i][j].Y].contactosR.push_back(coor);
          }
      }

      if((j+1!=tamAncho)&&(i-1!=-1)&&
        pir[nivel][i-1][j+1].Y!=-1 &&pir[nivel][i-1][j+1].X==-1 && pir[nivel][i][j].Y!=pir[nivel][i-1][j+1].Y){
        if(std::find(nodos[pir[nivel][i][j].Y].contactos.begin(),nodos[pir[nivel][i][j].Y].contactos.end(),
          pir[nivel][i-1][j+1].Y)==nodos[pir[nivel][i][j].Y].contactos.end()){
                nodos[pir[nivel][i][j].Y].contactos.push_back(pir[nivel][i-1][j+1].Y);
          }
        if(std::find(nodos[pir[nivel][i-1][j+1].Y].contactos.begin(),nodos[pir[nivel][i-1][j+1].Y].contactos.end(),
          pir[nivel][i][j].Y)==nodos[pir[nivel][i-1][j+1].Y].contactos.end()){
                nodos[pir[nivel][i-1][j+1].Y].contactos.push_back(pir[nivel][i][j].Y);
          }
      } else if((j+1!=tamAncho)&&(i-1!=-1)&&pir[nivel][i-1][j+1].Y!=-1 &&pir[nivel][i-1][j+1].X!=-1){
        coor.i=pir[nivel][i-1][j+1].X;
        coor.j=pir[nivel][i-1][j+1].Y;
        if(std::find(pir[nivel+1][pir[nivel][i-1][j+1].X][pir[nivel][i-1][j+1].Y].contactosV.begin(),pir[nivel+1][pir[nivel][i-1][j+1].X][pir[nivel][i-1][j+1].Y].contactosV.end(),
          pir[nivel][i][j].Y)==pir[nivel+1][pir[nivel][i-1][j+1].X][pir[nivel][i-1][j+1].Y].contactosV.end()){
                pir[nivel+1][pir[nivel][i-1][j+1].X][pir[nivel][i-1][j+1].Y].contactosV.push_back(pir[nivel][i][j].Y);
          }
       if(std::find_if(nodos[pir[nivel][i][j].Y].contactosR.begin(),nodos[pir[nivel][i][j].Y].contactosR.end(),
          CmpContactos(coor))==nodos[pir[nivel][i][j].Y].contactosR.end()){
                nodos[pir[nivel][i][j].Y].contactosR.push_back(coor);
          }
      } //else if
      } //else if
   } //for
  } //for
}

/******************************************************************************************

********************************************************************************************************/
void CalcularAdyacenciaVirtual(PIR ***pir, std::vector<NV> &nodos, std::vector<NV> nodosinf, int nivel, int solovirtual,int alto,int ancho){

register int i,j;
PUNTO coor;

for(i=0;i<(int)nodosinf.size();i++){
//Si dos nodos virtuales están en contacto en un nivel sus padres estarán en contacto
//en el nivel superior
   if(nodosinf[i].padre!=-1){
      for(j=0;j<(int)nodosinf[i].contactos.size();j++){
         if(nodosinf[nodosinf[i].contactos[j]].padre!=-1&&nodosinf[i].padre!=nodosinf[nodosinf[i].contactos[j]].padre){
         if(std::find(nodos[nodosinf[i].padre].contactos.begin(),nodos[nodosinf[i].padre].contactos.end(),
          nodosinf[nodosinf[i].contactos[j]].padre)==nodos[nodosinf[i].padre].contactos.end())
            nodos[nodosinf[i].padre].contactos.push_back(nodosinf[nodosinf[i].contactos[j]].padre);

         if(std::find(nodos[nodosinf[nodosinf[i].contactos[j]].padre].contactos.begin(),nodos[nodosinf[nodosinf[i].contactos[j]].padre].contactos.end(),
          nodosinf[i].padre)==nodos[nodosinf[nodosinf[i].contactos[j]].padre].contactos.end())
            nodos[nodosinf[nodosinf[i].contactos[j]].padre].contactos.push_back(nodosinf[i].padre);
         }//if
      } //for
      if(!solovirtual){
       for(j=0;j<(int)nodosinf[i].contactosR.size();j++){
          if(pir[nivel][(int)nodosinf[i].contactosR[j].i][(int)nodosinf[i].contactosR[j].j].X==-1&&
                pir[nivel][(int)nodosinf[i].contactosR[j].i][(int)nodosinf[i].contactosR[j].j].Y!=-1&&
                pir[nivel][(int)nodosinf[i].contactosR[j].i][(int)nodosinf[i].contactosR[j].j].Y!=nodosinf[i].padre){

          if(std::find(nodos[nodosinf[i].padre].contactos.begin(),nodos[nodosinf[i].padre].contactos.end(),
          pir[nivel][(int)nodosinf[i].contactosR[j].i][(int)nodosinf[i].contactosR[j].j].Y)==nodos[nodosinf[i].padre].contactos.end())
            nodos[nodosinf[i].padre].contactos.push_back(pir[nivel][(int)nodosinf[i].contactosR[j].i][(int)nodosinf[i].contactosR[j].j].Y);

         if(std::find(nodos[pir[nivel][(int)nodosinf[i].contactosR[j].i][(int)nodosinf[i].contactosR[j].j].Y].contactos.begin(),nodos[pir[nivel][(int)nodosinf[i].contactosR[j].i][(int)nodosinf[i].contactosR[j].j].Y].contactos.end(),
          nodosinf[i].padre)==nodos[pir[nivel][(int)nodosinf[i].contactosR[j].i][(int)nodosinf[i].contactosR[j].j].Y].contactos.end())
            nodos[pir[nivel][(int)nodosinf[i].contactosR[j].i][(int)nodosinf[i].contactosR[j].j].Y].contactos.push_back(nodosinf[i].padre);
         }//if
         else if(pir[nivel][(int)nodosinf[i].contactosR[j].i][(int)nodosinf[i].contactosR[j].j].X!=-1&&
                pir[nivel][(int)nodosinf[i].contactosR[j].i][(int)nodosinf[i].contactosR[j].j].Y!=-1){

                coor.i= pir[nivel][(int)nodosinf[i].contactosR[j].i][(int)nodosinf[i].contactosR[j].j].X;
                coor.j=pir[nivel][(int)nodosinf[i].contactosR[j].i][(int)nodosinf[i].contactosR[j].j].Y;

          if(std::find(pir[nivel+1][(int)coor.i][(int)coor.j].contactosV.begin(),
          pir[nivel+1][(int)coor.i][(int)coor.j].contactosV.end(),
          nodosinf[i].padre)==pir[nivel+1][(int)coor.i][(int)coor.j].contactosV.end())
            pir[nivel+1][(int)coor.i][(int)coor.j].contactosV.push_back(nodosinf[i].padre);

          if(std::find_if(nodos[nodosinf[i].padre].contactosR.begin(),nodos[nodosinf[i].padre].contactosR.end(),
          CmpContactos(coor))==nodos[nodosinf[i].padre].contactosR.end()){
                nodos[nodosinf[i].padre].contactosR.push_back(coor);
          }//if
         }//else if
       }//for
       }
   }//if
}//for
}
/***************************************************************************************************
GENERAR PARTE IRREGULAR:  Realiza la etapa de parent search e intralevel twinning entre nodos regulares.
También realiza las etapas de virtual parent search and virtual vertices linking  entre nodos virtuales.
*******************************************************************************************************/

void BIP::GenerarParteIrregular(int nivel, double umbral){
register int i,j,l;
double teta,H;
PUNTO p1,p2,medio;
std::vector<DIST> dist;
DIST aux_dist;
int lmin,lminV;
double distmin,distminV;
long int cont=0;
NV nodo;
std::vector<double> distV;
std::vector<int> pos;
std::vector<double> distR;
std::vector<PUNTO> posR;
PUNTO aux;
PUNTO lminV1;
int tamanoAlto;
int tamanoAncho;

//Se empieza con la parte de la jerarquía que contiene nodos regulares y virtuales
if(nivel<niveles-1){
tamanoAlto=TamNivelesAlto[nivel];
tamanoAncho=TamNivelesAncho[nivel];

if(nivel>0){
 //Lo primero es ver que nodos virtuales están en contacto

         CalcularAdyacenciaRegular(NodosV[nivel],pir,TamNivelesAlto[nivel-1],TamNivelesAncho[nivel-1],nivel-1);

        if (nivel>1)

         CalcularAdyacenciaVirtual(pir,NodosV[nivel],NodosV[nivel-1],nivel-1,0,TamNivelesAlto[nivel],TamNivelesAncho[nivel]);
}

//Un nodo regular sin padre busca entre sus vecinos regulares o irregulares el mas parecido a el y que cumpla el umbral
//y que tenga padre para unirse a el. Si no lo encuentra buscara entre sus vecinos regulares o irregulares sin padre
//para generar un nodo virtual

//Primero busco entre los regulares
//Primero en la vecindad 8
 for(i=0;i<TamNivelesAlto[nivel];i++)
        for(j=0;j<TamNivelesAncho[nivel];j++){

        dist.clear();

          if(pir[nivel][i][j].Homog==1&&pir[nivel][i][j].X==-1 && pir[nivel][i][j].Y==-1){
                if(((j-1)!=-1)&&
                (pir[nivel][i][j-1].Homog==1)){
                        aux_dist.dif=DisColor(pir[nivel][i][j],pir[nivel][i][j-1]);
                        aux_dist.x=i;
                        aux_dist.y=j-1;
                        dist.push_back(aux_dist);
                }

                if(((j-1)!=-1)&&((i-1)!=-1)
                &&(pir[nivel][i-1][j-1].Homog==1)){
                        aux_dist.dif=DisColor(pir[nivel][i][j],pir[nivel][i-1][j-1]);
                        aux_dist.x=i-1;
                        aux_dist.y=j-1;
                        dist.push_back(aux_dist);
                }

                if(((i-1)!=-1)
                &&(pir[nivel][i-1][j].Homog==1)){
                        aux_dist.dif=DisColor(pir[nivel][i][j],pir[nivel][i-1][j]);
                        aux_dist.x=i-1;
                        aux_dist.y=j;
                        dist.push_back(aux_dist);
                }
                if(((i-1)!=-1)&&((j+1)!=tamanoAncho)
                &&(pir[nivel][i-1][j+1].Homog==1)){
                        aux_dist.dif=DisColor(pir[nivel][i][j],pir[nivel][i-1][j+1]);
                        aux_dist.x=i-1;
                        aux_dist.y=j+1;
                        dist.push_back(aux_dist);
                }
                if(((j+1)!=tamanoAncho)
                &&(pir[nivel][i][j+1].Homog==1)){
                        aux_dist.dif=DisColor(pir[nivel][i][j],pir[nivel][i][j+1]);
                        aux_dist.x=i;
                        aux_dist.y=j+1;
                        dist.push_back(aux_dist);
                }
                if(((j+1)!=tamanoAncho) &&((i+1)!=tamanoAlto)
                &&(pir[nivel][i+1][j+1].Homog==1)){
                        aux_dist.dif=DisColor(pir[nivel][i][j],pir[nivel][i+1][j+1]);
                        aux_dist.x=i+1;
                        aux_dist.y=j+1;
                        dist.push_back(aux_dist);
                }
                if(((i+1)!=tamanoAlto)
                &&(pir[nivel][i+1][j].Homog==1)){
                        aux_dist.dif=DisColor(pir[nivel][i][j],pir[nivel][i+1][j]);
                        aux_dist.x=i+1;
                        aux_dist.y=j;
                        dist.push_back(aux_dist);
                }
                if(((j-1)!=-1)&&((i+1)!=tamanoAlto)
                &&(pir[nivel][i+1][j-1].Homog==1)){
                        aux_dist.dif=DisColor(pir[nivel][i][j],pir[nivel][i+1][j-1]);
                        aux_dist.x=i+1;
                        aux_dist.y=j-1;
                        dist.push_back(aux_dist);
                }
 //Ahora entre sus vecinos regulares que no estan en vecindad 8

       for (l=0;l<(int)pir[nivel][i][j].contactosR.size();l++){
                aux_dist.dif=DisColor(pir[nivel][i][j],
                        pir[nivel][pir[nivel][i][j].contactosR[l].i][pir[nivel][i][j].contactosR[l].j]);
                aux_dist.x=pir[nivel][i][j].contactosR[l].i;
                aux_dist.y=pir[nivel][i][j].contactosR[l].j;
                dist.push_back(aux_dist);
        }


          distmin=10000.0;
          lmin=-1;
          lminV=-1;


       /*Entre los vecinos con padre buscamos el que tiene distancia
       minima en nivel de color con el nodo y cumple que esta distancia
       es menor que el umbral, y pegamos el nodo al padre del vecino*/


          for (l=0;l<(int)dist.size();l++){

             //Aqui busca los regulares con padre
               if(pir[nivel][dist[l].x][dist[l].y].Y!=-1 && dist[l].dif<distmin &&
                   dist[l].dif<umbral){
                   distmin=dist[l].dif;
                   lmin=l;

               }
          }


          //Ahora voy a buscar entre los irregulares

          lminV=-1;
          distminV=10000.0;
         for(l=0; l<(int)pir[nivel][i][j].contactosV.size();l++){
                   if((NodosV[nivel][pir[nivel][i][j].contactosV[l]].padre!=-1&&(DisColor(pir[nivel][i][j],NodosV[nivel][pir[nivel][i][j].contactosV[l]]))<distminV)&&
                   (DisColor(pir[nivel][i][j],NodosV[nivel][pir[nivel][i][j].contactosV[l]]))<umbral){
                        distminV=DisColor(pir[nivel][i][j],NodosV[nivel][pir[nivel][i][j].contactosV[l]]);
                        lminV=pir[nivel][i][j].contactosV[l];
                   }
          }

          /*Se pega al padre del vecino*/
           if (lmin!=-1&&lminV!=-1){
                if (distmin<=distminV){

                      pir[nivel][i][j].X=pir[nivel][dist[lmin].x][dist[lmin].y].X;
                      pir[nivel][i][j].Y=pir[nivel][dist[lmin].x][dist[lmin].y].Y;


                      if(pir[nivel][i][j].X==-1 && pir[nivel][i][j].Y!=-1){//El padre es nodo virtual

                        nodo=CrearNodoVirtual(pir[nivel][i][j],NodosV[nivel+1][pir[nivel][i][j].Y]);
                        NodosV[nivel+1][pir[nivel][i][j].Y].Area= nodo.Area;
                        NodosV[nivel+1][pir[nivel][i][j].Y].C=nodo.C;
                        NodosV[nivel+1][pir[nivel][i][j].Y].S=nodo.S;
                        NodosV[nivel+1][pir[nivel][i][j].Y].I=nodo.I;


                      }else if (pir[nivel][i][j].X!=-1 &&pir[nivel][i][j].Y!=-1){  //el padre es un nodo de la estructura

pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].I=
                        ((pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].I
                        *pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].Area)
                        +(pir[nivel][i][j].I*pir[nivel][i][j].Area))/(pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].Area
                        +pir[nivel][i][j].Area);

                         teta=pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].C*2*3.1415/255;
                                p1.i=(pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].S*cos(teta));
                                p1.j=(pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].S*sin(teta));

                                teta=pir[nivel][i][j].C*2*3.1415/255;
                                p2.i=(pir[nivel][i][j].S*cos(teta));
                                p2.j=(pir[nivel][i][j].S*sin(teta));

                                medio.i=(p1.i+p2.i)/2;
                                medio.j=(p1.j+p2.j)/2;

                                if(medio.j==0 && medio.i==0) H=0;
                                else H=(atan2(medio.j,medio.i))*180/3.1415;

                                if(H<0) H=360+H;
                                H=H*255/360;

                                pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].C=(unsigned char)H;

                                pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].S=(unsigned char)sqrt((medio.i*medio.i)+(medio.j*medio.j));


                        pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].Area=
                        pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].Area
                        +pir[nivel][i][j].Area;

                      }
                }else {
                        pir[nivel][i][j].X=-1;
                        pir[nivel][i][j].Y=NodosV[nivel][lminV].padre;
                        nodo=CrearNodoVirtual(pir[nivel][i][j],NodosV[nivel+1][pir[nivel][i][j].Y]);
                        NodosV[nivel+1][pir[nivel][i][j].Y].Area= nodo.Area;
                        NodosV[nivel+1][pir[nivel][i][j].Y].C=nodo.C;
                        NodosV[nivel+1][pir[nivel][i][j].Y].S=nodo.S;
                        NodosV[nivel+1][pir[nivel][i][j].Y].I=nodo.I;

                }
           }else if(lminV==-1 &&lmin!=-1){

                      pir[nivel][i][j].X=pir[nivel][dist[lmin].x][dist[lmin].y].X;
                      pir[nivel][i][j].Y=pir[nivel][dist[lmin].x][dist[lmin].y].Y;


                      if(pir[nivel][i][j].X==-1 && pir[nivel][i][j].Y!=-1){//El padre es nodo virtual

                       nodo=CrearNodoVirtual(pir[nivel][i][j],NodosV[nivel+1][pir[nivel][i][j].Y]);
                       NodosV[nivel+1][pir[nivel][i][j].Y].Area= nodo.Area;
                       NodosV[nivel+1][pir[nivel][i][j].Y].C=nodo.C;
                       NodosV[nivel+1][pir[nivel][i][j].Y].S=nodo.S;
                       NodosV[nivel+1][pir[nivel][i][j].Y].I=nodo.I;

                      }else if (pir[nivel][i][j].X!=-1 &&pir[nivel][i][j].Y!=-1){  //el padre es un nodo de la estructura

                        pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].I=
                        ((pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].I*pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].Area)
                        +(pir[nivel][i][j].I*pir[nivel][i][j].Area))/(pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].Area
                        +pir[nivel][i][j].Area);

                        teta=pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].C*2*3.1415/255;
                                p1.i=(pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].S*cos(teta));
                                p1.j=(pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].S*sin(teta));

                                teta=pir[nivel][i][j].C*2*3.1415/255;
                                p2.i=(pir[nivel][i][j].S*cos(teta));
                                p2.j=(pir[nivel][i][j].S*sin(teta));

                                medio.i=(p1.i+p2.i)/2;
                                medio.j=(p1.j+p2.j)/2;

                                if(medio.j==0 && medio.i==0) H=0;
                                else H=(atan2(medio.j,medio.i))*180/3.1415;

                                if(H<0) H=360+H;
                                H=H*255/360;

                                pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].C=(unsigned char)H;

                                pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].S=(unsigned char)sqrt((medio.i*medio.i)+(medio.j*medio.j));

                        pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].Area=
                        pir[nivel+1][pir[nivel][i][j].X][pir[nivel][i][j].Y].Area
                        +pir[nivel][i][j].Area;


                      }
           }else if(lmin==-1&&lminV!=-1){
                        pir[nivel][i][j].X=-1;
                        pir[nivel][i][j].Y=NodosV[nivel][lminV].padre;
                        nodo=CrearNodoVirtual(pir[nivel][i][j],NodosV[nivel+1][pir[nivel][i][j].Y]);
                        NodosV[nivel+1][pir[nivel][i][j].Y].Area= nodo.Area;
                        NodosV[nivel+1][pir[nivel][i][j].Y].C=nodo.C;
                        NodosV[nivel+1][pir[nivel][i][j].Y].S=nodo.S;
                        NodosV[nivel+1][pir[nivel][i][j].Y].I=nodo.I;


           } else if (lmin==-1 && lminV==-1){

             distmin=10000.0;
             lmin=-1;

             for (l=0;l<(int)dist.size();l++){
                //Aqui como no encontro nodo con padre, busca uno sin padre
                 if(pir[nivel][dist[l].x][dist[l].y].Y==-1 && dist[l].dif<distmin &&
                    dist[l].dif<umbral){
                     distmin=dist[l].dif;
                     lmin=l;
                 }
             }

          lminV=-1;
          distminV=10000.0;
         for(l=0; l<(int)pir[nivel][i][j].contactosV.size();l++){
                   if((NodosV[nivel][pir[nivel][i][j].contactosV[l]].padre==-1&&(DisColor(pir[nivel][i][j],NodosV[nivel][pir[nivel][i][j].contactosV[l]]))<distminV)&&
                   (DisColor(pir[nivel][i][j],NodosV[nivel][pir[nivel][i][j].contactosV[l]]))<umbral){
                        distminV=DisColor(pir[nivel][i][j],NodosV[nivel][pir[nivel][i][j].contactosV[l]]);
                        lminV=pir[nivel][i][j].contactosV[l];
                   }
          }

          if (lmin!=-1 && lminV!=-1){
                if (distmin<=distminV){
                   nodo=CrearNodoVirtual(pir[nivel][i][j],
                               pir[nivel][dist[lmin].x][dist[lmin].y]);
                   NodosV[nivel+1].push_back(nodo);
                   pir[nivel][i][j].X=-1;
                   pir[nivel][i][j].Y=cont;
                   pir[nivel][dist[lmin].x][dist[lmin].y].X=-1;
                   pir[nivel][dist[lmin].x][dist[lmin].y].Y=cont;
                   cont++;

                }else {
                   nodo=CrearNodoVirtual(pir[nivel][i][j],
                               NodosV[nivel][lminV]);
                   NodosV[nivel+1].push_back(nodo);
                   pir[nivel][i][j].X=-1;
                   pir[nivel][i][j].Y=cont;
                   NodosV[nivel][lminV].padre=cont;
                   cont++;
                }

          } else if(lmin!=-1&&lminV==-1){
                   nodo=CrearNodoVirtual(pir[nivel][i][j],
                               pir[nivel][dist[lmin].x][dist[lmin].y]);
                   NodosV[nivel+1].push_back(nodo);
                   pir[nivel][i][j].X=-1;
                   pir[nivel][i][j].Y=cont;
                   pir[nivel][dist[lmin].x][dist[lmin].y].X=-1;
                   pir[nivel][dist[lmin].x][dist[lmin].y].Y=cont;
                   cont++;
          } else if(lmin==-1&&lminV!=-1){
                   nodo=CrearNodoVirtual(pir[nivel][i][j],
                               NodosV[nivel][lminV]);
                   NodosV[nivel+1].push_back(nodo);
                   pir[nivel][i][j].X=-1;
                   pir[nivel][i][j].Y=cont;
                   NodosV[nivel][lminV].padre=cont;
                   cont++;
       }
     }
  }
 }

//Un nodo virtual sin padre busca entre sus vecinos virtuales el mas parecido a el y que cumpla el umbral
//y que tenga padre para unirse a el. Si no lo encuentra buscara entre sus vecinos virtuales sin padre
//para generar un nodo virtual


 if(nivel>0){
 //Lo primero es ver que nodos virtuales están en contacto

        for(i=0;i<(int)NodosV[nivel].size();i++){
                if (NodosV[nivel][i].padre==-1){

                distV.clear();
                pos.clear();
                    for(j=0;j<(int)NodosV[nivel][i].contactos.size();j++){
                        distV.push_back(DisColor(NodosV[nivel][i],NodosV[nivel][NodosV[nivel][i].contactos[j]]));
                        pos.push_back(NodosV[nivel][i].contactos[j]);
                    }

                distR.clear();
                posR.clear();

                     for(j=0;j<(int)NodosV[nivel][i].contactosR.size();j++){

                        distR.push_back(DisColor(NodosV[nivel][i],pir[nivel][NodosV[nivel][i].contactosR[j].i][NodosV[nivel][i].contactosR[j].j]));
                        aux.i=NodosV[nivel][i].contactosR[j].i;
                        aux.j=NodosV[nivel][i].contactosR[j].j;
                        posR.push_back(aux);
                     }

                distmin=10000.0;
                lmin=-1;
                distminV=10000.0;
                lminV1.i=-1;
                lminV1.j=-1;
                       for(l=0;l<(int)distV.size();l++){
                        //Busca entre los no-huérfanos
                                if(distV[l]<distmin && NodosV[nivel][pos[l]].padre!=-1 &&
                                distV[l]<umbral){
                                     distmin=distV[l];
                                     lmin=pos[l];
                                }
                        }

                       for(l=0;l<(int)distR.size();l++){

                                if(distR[l]<distmin && pir[nivel][posR[l].i][posR[l].j].X==-1
                                &&pir[nivel][posR[l].i][posR[l].j].Y!=-1 &&
                                distR[l]<umbral){
                                     distminV=distR[l];
                                     lminV1.i=posR[l].i;
                                     lminV1.j=posR[l].j;
                                }
                        }

                        if (lmin!=-1 && lminV1.i!=-1){
                                if (distmin<=distminV){
                                    NodosV[nivel][i].padre=NodosV[nivel][lmin].padre;
                                    //Esto es para la media de color
                                    nodo=CrearNodoVirtual(NodosV[nivel][i],NodosV[nivel+1][NodosV[nivel][i].padre]);
                                    NodosV[nivel+1][NodosV[nivel][i].padre].Area= nodo.Area;
                                    NodosV[nivel+1][NodosV[nivel][i].padre].C=nodo.C;
                                    NodosV[nivel+1][NodosV[nivel][i].padre].S=nodo.S;
                                    NodosV[nivel+1][NodosV[nivel][i].padre].I=nodo.I;

                                }else{
                                    NodosV[nivel][i].padre=pir[nivel][lminV1.i][lminV1.j].Y;
                                    nodo=CrearNodoVirtual(NodosV[nivel][i],NodosV[nivel+1][NodosV[nivel][i].padre]);
                                    NodosV[nivel+1][NodosV[nivel][i].padre].Area=nodo.Area;
                                    NodosV[nivel+1][NodosV[nivel][i].padre].C=nodo.C;
                                    NodosV[nivel+1][NodosV[nivel][i].padre].S=nodo.S;
                                    NodosV[nivel+1][NodosV[nivel][i].padre].I=nodo.I;

                                }
                        }else if(lmin!=-1&&lminV1.i==-1){
                                    NodosV[nivel][i].padre=NodosV[nivel][lmin].padre;
                                    //Esto es para la media de color
                                    nodo=CrearNodoVirtual(NodosV[nivel][i],NodosV[nivel+1][NodosV[nivel][i].padre]);
                                    NodosV[nivel+1][NodosV[nivel][i].padre].Area= nodo.Area;
                                    NodosV[nivel+1][NodosV[nivel][i].padre].C=nodo.C;
                                    NodosV[nivel+1][NodosV[nivel][i].padre].S=nodo.S;
                                    NodosV[nivel+1][NodosV[nivel][i].padre].I=nodo.I;

                        } else if(lmin==-1&&lminV1.i!=-1){
                                    NodosV[nivel][i].padre=pir[nivel][lminV1.i][lminV1.j].Y;
                                    nodo=CrearNodoVirtual(NodosV[nivel][i],NodosV[nivel+1][NodosV[nivel][i].padre]);
                                    NodosV[nivel+1][NodosV[nivel][i].padre].Area=nodo.Area;
                                    NodosV[nivel+1][NodosV[nivel][i].padre].C=nodo.C;
                                    NodosV[nivel+1][NodosV[nivel][i].padre].S=nodo.S;
                                    NodosV[nivel+1][NodosV[nivel][i].padre].I=nodo.I;


                        }else if (lmin==-1&&lminV1.i==-1){
                               distmin=10000.0;
                               distminV=10000.0;
                               lminV1.i=-1;


                        for(l=0;l<(int)distV.size();l++){
                        //Busca entre los huérfanos
                                 if(distV[l]<distmin && NodosV[nivel][pos[l]].padre==-1 &&
                                 distV[l]<umbral){
                                        distmin=distV[l];
                                        lmin=pos[l];
                                 }
                                }

                                for(l=0;l<(int)distR.size();l++){
                                        if(distR[l]<distmin && pir[nivel][posR[l].i][posR[l].j].X==-1
                                        &&pir[nivel][posR[l].i][posR[l].j].Y==-1 &&
                                        distR[l]<umbral){
                                                distminV=distR[l];
                                                lminV1.i=posR[l].i;
                                                lminV1.j=posR[l].j;
                                        }
                                }

                                if (lmin!=-1&&lminV1.i!=-1){
                                        if(distmin<=distminV){
                                              nodo=CrearNodoVirtual(NodosV[nivel][i],NodosV[nivel][lmin]);
                                              NodosV[nivel+1].push_back(nodo);
                                              NodosV[nivel][i].padre=cont;
                                              NodosV[nivel][lmin].padre=cont;
                                              cont++;
                                        } else{
                                              nodo=CrearNodoVirtual(NodosV[nivel][i],pir[nivel][lminV1.i][lminV1.j]);
                                              NodosV[nivel+1].push_back(nodo);
                                              NodosV[nivel][i].padre=cont;
                                              pir[nivel][lminV1.i][lminV1.j].X=-1;
                                              pir[nivel][lminV1.i][lminV1.j].Y=cont;
                                              cont++;
                                        }
                                }else if(lmin!=-1&&lminV1.i==-1){
                                       nodo=CrearNodoVirtual(NodosV[nivel][i],NodosV[nivel][lmin]);
                                       NodosV[nivel+1].push_back(nodo);
                                       NodosV[nivel][i].padre=cont;
                                       NodosV[nivel][lmin].padre=cont;
                                       cont++;
                                }else if(lmin==-1&&lminV1.i!=-1){
                                       nodo=CrearNodoVirtual(NodosV[nivel][i],pir[nivel][lminV1.i][lminV1.j]);
                                       NodosV[nivel+1].push_back(nodo);
                                       NodosV[nivel][i].padre=cont;
                                       pir[nivel][lminV1.i][lminV1.j].X=-1;
                                       pir[nivel][lminV1.i][lminV1.j].Y=cont++;

                                }else if (lmin==-1&&lminV1.i==-1){
                                        nodo.C=NodosV[nivel][i].C;
                                        nodo.S=NodosV[nivel][i].S;
                                        nodo.I=NodosV[nivel][i].I;
                                        nodo.Area=NodosV[nivel][i].Area;
                                        nodo.Clase=-1;
                                        nodo.padre=-1;
                                        NodosV[nivel+1].push_back(nodo);
                                        NodosV[nivel][i].padre=cont;
                                        cont++;
                                }

                        }


                }
        }
 }

 //A partir de aqui se esta teniendo en cuenta solo la parte alta de la jerarquía
 //donde solo hay nodos virtuales

 }else{

        CalcularAdyacenciaVirtual(pir,NodosV[nivel],NodosV[nivel-1],nivel-1,1,TamNivelesAlto[nivel],TamNivelesAncho[nivel]);


        for(i=0;i<(int)NodosV[nivel].size();i++){
                if (NodosV[nivel][i].padre==-1){
                distV.clear();
                pos.clear();
                    for(j=0;j<(int)NodosV[nivel][i].contactos.size();j++){
                        distV.push_back(DisColor(NodosV[nivel][i],NodosV[nivel][NodosV[nivel][i].contactos[j]]));
                        pos.push_back(NodosV[nivel][i].contactos[j]);

                    }
                        distmin=10000.0;
                        lmin=-1;
                        for(l=0;l<(int)distV.size();l++){
                                if(distV[l]<distmin && NodosV[nivel][pos[l]].padre!=-1 &&
                                DisColor(NodosV[nivel][i],NodosV[nivel][pos[l]])<umbral){
                                     distmin=distV[l];
                                     lmin=pos[l];
                                }
                        }
                        if(lmin!=-1){
                                NodosV[nivel][i].padre=NodosV[nivel][lmin].padre;
                                nodo=CrearNodoVirtual(NodosV[nivel][i],NodosV[nivel+1][NodosV[nivel][i].padre]);
                                NodosV[nivel+1][NodosV[nivel][i].padre].Area= nodo.Area;
                                NodosV[nivel+1][NodosV[nivel][i].padre].C=nodo.C;
                                NodosV[nivel+1][NodosV[nivel][i].padre].S=nodo.S;
                                NodosV[nivel+1][NodosV[nivel][i].padre].I=nodo.I;

                        }
                        else{
                          distmin=10000.0;
                          lmin=-1;
                        for(l=0;l<(int)distV.size();l++){
                                if(distV[l]<distmin && NodosV[nivel][pos[l]].padre==-1 &&
                                DisColor(NodosV[nivel][i],NodosV[nivel][pos[l]])<umbral){
                                     distmin=distV[l];
                                     lmin=pos[l];
                                }
                        }
                                if(lmin!=-1){
                                    nodo=CrearNodoVirtual(NodosV[nivel][i],
                                        NodosV[nivel][lmin]);
                                    NodosV[nivel+1].push_back(nodo);
                                    NodosV[nivel][i].padre=cont;
                                    NodosV[nivel][lmin].padre=cont;
                                    cont++;
                                }
                                else{
                                        nodo.C=NodosV[nivel][i].C;
                                        nodo.S=NodosV[nivel][i].S;
                                        nodo.I=NodosV[nivel][i].I;
                                        nodo.Area=NodosV[nivel][i].Area;
                                        nodo.Clase=-1;
                                        nodo.padre=-1;
                                        NodosV[nivel+1].push_back(nodo);
                                        NodosV[nivel][i].padre=cont;
                                        cont++;
                                }

                        }

                }
        }
  }
}


void BoundingBox(PIR ***pir,BOX *cajas, int clases, int alto, int ancho){

register int i,j,clase;

//Inicialización de las cajas

for(i=0;i<clases;i++){
    cajas[i].imin=alto;
    cajas[i].jmin=ancho;
    cajas[i].imax=-1;
    cajas[i].jmax=-1;
    cajas[i].area=0;
}

//Calculo de las cajas

for(i=0;i<alto;i++){
  for(j=0;j<ancho;j++){
    clase=pir[0][i][j].Clase;
   if(clase!=-1){

     cajas[clase].area=cajas[clase].area+1;

     if(cajas[clase].imin>i)
         cajas[clase].imin=i;

     if(cajas[clase].jmin>j)
         cajas[clase].jmin=j;

     if(cajas[clase].imax<i)
         cajas[clase].imax=i;

     if(cajas[clase].jmax<j)
         cajas[clase].jmax=j;

      //Se le asigna a cada caja el color del padre absoluto de su clase

         cajas[clase].C=pir[0][i][j].C;
         cajas[clase].S=pir[0][i][j].S;
         cajas[clase].I=pir[0][i][j].I;
   }
  }
}
}


void CalcContactos(int **contactos,PIR ***pir, int altoim, int anchoim){

register int i,j;

//Se recorre la base de la pirámide, y cada píxel pregunta a sus vecinos
//si tiene una clase diferente a la suya

for(i=0;i<altoim;i++){
   for(j=0;j<anchoim;j++){

      if((j-1!=-1)&& (pir[0][i][j].Clase!=pir[0][i][j-1].Clase)&&
        pir[0][i][j].Clase!=-1 &&pir[0][i][j-1].Clase!=-1){

         contactos[pir[0][i][j].Clase][pir[0][i][j-1].Clase]++;
         contactos[pir[0][i][j-1].Clase][pir[0][i][j].Clase]++;

      }

      if((j-1!=-1)&&(i-1!=-1)&&(pir[0][i][j].Clase!=pir[0][i-1][j-1].Clase)&&
        pir[0][i][j].Clase!=-1 &&pir[0][i-1][j-1].Clase!=-1){

         contactos[pir[0][i][j].Clase][pir[0][i-1][j-1].Clase]++;
         contactos[pir[0][i-1][j-1].Clase][pir[0][i][j].Clase]++;

      }

      if((i-1!=-1)&& (pir[0][i][j].Clase!=pir[0][i-1][j].Clase)&&
        pir[0][i][j].Clase!=-1 &&pir[0][i-1][j].Clase!=-1){

         contactos[pir[0][i][j].Clase][pir[0][i-1][j].Clase]++;
         contactos[pir[0][i-1][j].Clase][pir[0][i][j].Clase]++;

      }

      if((j+1!=anchoim)&&(i-1!=-1)&&(pir[0][i][j].Clase!=pir[0][i-1][j+1].Clase)&&
        pir[0][i][j].Clase!=-1 &&pir[0][i-1][j+1].Clase!=-1){

         contactos[pir[0][i][j].Clase][pir[0][i-1][j+1].Clase]++;
         contactos[pir[0][i-1][j+1].Clase][pir[0][i][j].Clase]++;

      }
   }
}

}


void CalcContactosCanny(int **contactos,int **contactosCanny,PIR ***pir,
                        int **im,int altoim, int anchoim){

register int i,j;

//Se recorre la base de la pirámide, y cada píxel pregunta a sus vecinos
//si tiene una clase diferente a la suya

for(i=0;i<altoim;i++){
   for(j=0;j<anchoim;j++){

      if((j-1!=-1)&& (pir[0][i][j].Clase!=pir[0][i][j-1].Clase)&&
        pir[0][i][j].Clase!=-1 &&pir[0][i][j-1].Clase!=-1){

         contactos[pir[0][i][j].Clase][pir[0][i][j-1].Clase]++;
         contactos[pir[0][i][j-1].Clase][pir[0][i][j].Clase]++;

         if (im[i][j]==255) {
                contactosCanny[pir[0][i][j].Clase][pir[0][i][j-1].Clase]++;
                contactosCanny[pir[0][i][j-1].Clase][pir[0][i][j].Clase]++;
         }
      }

      if((j-1!=-1)&&(i-1!=-1)&&(pir[0][i][j].Clase!=pir[0][i-1][j-1].Clase)&&
        pir[0][i][j].Clase!=-1 &&pir[0][i-1][j-1].Clase!=-1){

         contactos[pir[0][i][j].Clase][pir[0][i-1][j-1].Clase]++;
         contactos[pir[0][i-1][j-1].Clase][pir[0][i][j].Clase]++;

         if (im[i][j]==255) {
                contactosCanny[pir[0][i][j].Clase][pir[0][i-1][j-1].Clase]++;
                contactosCanny[pir[0][i-1][j-1].Clase][pir[0][i][j].Clase]++;
         }
      }

      if((i-1!=-1)&& (pir[0][i][j].Clase!=pir[0][i-1][j].Clase)&&
        pir[0][i][j].Clase!=-1 &&pir[0][i-1][j].Clase!=-1){

         contactos[pir[0][i][j].Clase][pir[0][i-1][j].Clase]++;
         contactos[pir[0][i-1][j].Clase][pir[0][i][j].Clase]++;

         if (im[i][j]==255) {
                contactosCanny[pir[0][i][j].Clase][pir[0][i-1][j].Clase]++;
                contactosCanny[pir[0][i-1][j].Clase][pir[0][i][j].Clase]++;
         }

      }

      if((j+1!=anchoim)&&(i-1!=-1)&&(pir[0][i][j].Clase!=pir[0][i-1][j+1].Clase)&&
        pir[0][i][j].Clase!=-1 &&pir[0][i-1][j+1].Clase!=-1){

         contactos[pir[0][i][j].Clase][pir[0][i-1][j+1].Clase]++;
         contactos[pir[0][i-1][j+1].Clase][pir[0][i][j].Clase]++;

         if (im[i][j]==255) {
                contactosCanny[pir[0][i][j].Clase][pir[0][i-1][j+1].Clase]++;
                contactosCanny[pir[0][i-1][j+1].Clase][pir[0][i][j].Clase]++;
         }

      }
   }
}

}

void Fusionar(FUS *fus,int i, int j, BOX *cajas){

//Para las clases de area <100 (se les asigna el color de la clase grande)

    fus[i].I=fus[j].I;
    fus[i].C=fus[j].C;
    fus[i].S=fus[j].S;

 //Actualiza el valor de color de las cajas de las clases que se fusionan

 cajas[i].C=fus[i].C;
 cajas[i].C=fus[i].C;
 cajas[i].S=fus[i].S;
 cajas[j].S=fus[i].S;
 cajas[i].I=fus[i].I;
 cajas[j].I=fus[i].I;
}


void BIP::FusionAreasPequenas(){

FUS *fusiones; //Array para almacenar las fusiones

BOX *cajas;

cajas=new BOX [ContClase];

register int i,j;
int cont1;

register int cont;

int **contactos;


double *colores;  //Array para almacenar la diferencia de color de una clase
                  // con las clases con las que tiene contacto

int colormin, posmin;

//Reserva de memoria para el array de colores

colores= new double[ContClase];

//Reserva de memoria para el array de fusiones

fusiones=new FUS [ContClase];

//Reserva de memoria para la matriz de contactos entre clases

contactos= new int* [ContClase];
for(i=0;i<ContClase;i++)
   contactos[i]= new int[ContClase];


for(i=0;i<ContClase;i++){
  for(j=0;j<ContClase;j++){
      contactos[i][j]=0;
  }
}


BoundingBox(pir,cajas,ContClase,altoim,anchoim);

//Se inicializa el array de fusiones

for (i=0;i<ContClase;i++){
  fusiones[i].clase=-1;
  fusiones[i].C=cajas[i].C;
  fusiones[i].S=cajas[i].S;
  fusiones[i].I=cajas[i].I;
  fusiones[i].Area=cajas[i].area;
}

//Calculo de contactos entre las clases de la base de la piramide

CalcContactos(contactos,pir,altoim,anchoim);

//Miramos para cada clase con área <AREAMIN su relación con las demás

cont=0;

for (i=0;i<ContClase;i++){

 if(fusiones[i].Area<AREAMIN){

 //Inicialización del array de colores
   for(j=0;j<ContClase;j++){

       colores[j]=10000.0;

   }

   for(j=0;j<ContClase; j++){

    //Si dos clases están en contacto calculamos su diferencia de color

     if(i!=j && contactos[i][j]!=0 && fusiones[j].Area>=AREAMIN){

        colores[j]=DisColor(fusiones[i],fusiones[j]);
     }
    }

    colormin=10000.0;
    posmin=-1;

    //Buscamos la clase con menor diferencia de color
    for(j=0;j<ContClase;j++){

       if(colores[j]<colormin){

         colormin=colores[j];
         posmin=j;

       }
    }

    if(colormin<10000.0){
      //Si la primera no está fusionada con nadie

       //if(fusiones[i].clase==-1){

      //Si la segunda tampoco está fusionada con nadie se fusionan las dos
      //asignandoles un nuevo valor de clase (el mismo a las dos) en el array
      //de fusiones

      if(fusiones[posmin].clase==-1){
         fusiones[i].clase=cont;
         fusiones[i].Area=AREAMIN;
         fusiones[posmin].clase=cont;
         Fusionar(fusiones,i,posmin,cajas);
         cont++;

      //Si la segunda si está fusionada, se le da a la primera el valor de clase
      // de la segunda y también su color

      }else{
          fusiones[i].clase=fusiones[posmin].clase;
          fusiones[i].Area=AREAMIN;
          fusiones[i].C=fusiones[posmin].C;
          fusiones[i].S=fusiones[posmin].S;
          fusiones[i].I=fusiones[posmin].I;

      }//if else
     //if
 }//if color
 }//if area<AREAMIN
} //for

// A las clases sin fusionar se les da un nuevo identificador de clase y
// se mantiene su color

for (i=0;i<ContClase;i++){
   if(fusiones[i].clase==-1){
      fusiones[i].clase=cont;
      fusiones[i].C=cajas[i].C;
      fusiones[i].S=cajas[i].S;
      fusiones[i].I=cajas[i].I;
      cont++;
   }
}


//Se actualizan los valores de clase y el color en la base de la piramide

for(i=0;i<altoim;i++){
  for(j=0;j<anchoim;j++){
    if(pir[0][i][j].Clase!=-1) {
       cont1=pir[0][i][j].Clase;
       pir[0][i][j].Clase=fusiones[cont1].clase;
       pir[0][i][j].C=fusiones[cont1].C;
       pir[0][i][j].S=fusiones[cont1].S;
       pir[0][i][j].I=fusiones[cont1].I;
   }
}
}

ContClase=cont;

}



void BIP::Segmentar(double umbral){


register int i,j;
int regular;


for(i=0;i<niveles-1;i++) {
        regular=GenerarParteRegular(i);
        GenerarParteIrregular(i,umbral);
        if (NodosV[i].size()==NodosV[i+1].size() && regular==-1){
         numNiveles=i+1;
	 break;
         }
}

for(i=niveles-1; i<nivelesI-1&&NodosV[i].size()!=NodosV[i-1].size(); i++){

       GenerarParteIrregular(i,umbral);
        numNiveles=i+1;
	}

Clasificar();
LLevarColoresArriba();

//FusionAreasPequenas();

}



void BIP::SegmentarContrasteColor(IMAGEN_CANNY canny,double umbral, double alfa, double beta){

int i,j,k;
int cont;

int **contactos;
int **contactosCanny;

 int *cont1;

cajas=new BOX[ContClase];

for(int i=0;i<ContClase; i++)
  cajas[i].contraste=new double[ContClase];


BoundingBox(pir,cajas,ContClase, canny.alto, canny.ancho);

contactos=new int* [ContClase];
  for(i=0;i<ContClase;i++)
    contactos[i]=new int[ContClase];

contactosCanny=new int* [ContClase];
  for(i=0;i<ContClase;i++)
    contactosCanny[i]=new int[ContClase];

for (i=0;i<ContClase;i++)
        for(j=0;j<ContClase; j++){
                contactos[i][j]=0;
                contactosCanny[i][j]=0;
                cajas[i].contraste[j]=0;
        }

CalcContactosCanny(contactos,contactosCanny,pir,canny.im,canny.alto,canny.ancho);

//Voy a calcular primero el perimetro de cada clase

for(i=0;i<ContClase; i++){
   cont=0;
   for(j=0;j<ContClase;j++){
     if(contactos[i][j]!=0 && i!=j ){
        cont=cont+contactos[i][j];
     }
   }

   if(cajas[i].imin==0){
     for(k=cajas[i].jmin;k<=cajas[i].jmax;k++){

       if(pir[0][0][k].Clase==i)
	 cont++;
     }
   }
   if(cajas[i].imax==altoim-1){
     for(k=cajas[i].jmin;k<=cajas[i].jmax;k++){
       if(pir[0][altoim-1][k].Clase==i)
	 cont++;
     }
   }
   if(cajas[i].jmin==0){
     for(k=cajas[i].imin;k<=cajas[i].imax;k++){
       if(pir[0][k][0].Clase==i)
	 cont++;
     }
   }
   if(cajas[i].jmax==anchoim-1){
     for(k=cajas[i].imin;k<=cajas[i].imax;k++){
       if(pir[0][k][anchoim-1].Clase==i)
	 cont++;
     }
   }
   cajas[i].perimetro=cont;
}

for(i=0;i<ContClase;i++){
   cont=0;
   for(j=0;j<ContClase;j++){
     if(contactos[i][j]!=0 && i!=j && cajas[i].contraste[j]==0){
       cajas[i].contraste[j]=DisColor(cajas[i],cajas[j]);
       cajas[j].contraste[i]=cajas[i].contraste[j];
       cajas[i].contraste[j]=(cajas[i].contraste[j]*cajas[i].perimetro)/(contactosCanny[i][j]*alfa+beta*(contactos[i][j]-contactosCanny[i][j]));
       cajas[j].contraste[i]=(cajas[j].contraste[i]*cajas[j].perimetro)/(contactosCanny[i][j]*alfa+beta*(contactos[i][j]-contactosCanny[i][j]));
     }
   }
}


//Voy a montar aqui una piramide nueva en la cual la base sean nodos nuevos virtuales

numNivelesC=0;

GenerarBaseC(contactos,contactosCanny);
numNivelesC++;

GenerarNivelC(0,umbral,alfa,beta,contactos,contactosCanny);
numNivelesC++;

for(i=numNivelesC-1; i<nivelesC-2&&NodosC[i].size()!=NodosC[i-1].size(); i++){
	GenerarNivelC(i,umbral,alfa,beta,contactos,contactosCanny);
	numNivelesC++;
}

ClasificarC();

for(i=0;i<canny.alto; i++){
	for(j=0;j<canny.ancho;j++){
		pir[0][i][j].Clase=NodosC[0][pir[0][i][j].Clase].Clase;
		pir[0][i][j].C=padres[pir[0][i][j].Clase].C;
		pir[0][i][j].S=padres[pir[0][i][j].Clase].S;
		pir[0][i][j].I=padres[pir[0][i][j].Clase].I;

	}
}

/*for(int k=niveles-1; k>0; k--){
   for(i=0;i<TamNivelesAlto[k];i++)
     for(j=0;j<TamNivelesAncho[k];j++){
       if (pir[k][i][j].Clase!=-1){
		 pir[k][i][j].Clase=NodosC[0][pir[k][i][j].Clase].Clase;
		 pir[k][i][j].C=padres[pir[k][i][j].Clase].C;
		 pir[k][i][j].S=padres[pir[k][i][j].Clase].S;
		 pir[k][i][j].I=padres[pir[k][i][j].Clase].I;
       }
     }
}
*/

}

void BIP::PerceptualSegmentation(IMAGEN_ENT entrada,double umbral,double umbral1, IMAGEN_CANNY canny,double alfa, double beta, IMAGEN_SEG *im_seg){


for(int i=0;i<entrada.alto;i++)
        for(int j=0;j<entrada.ancho;j++){
                pir[0][i][j].C=entrada.im[i][j].C;
                pir[0][i][j].S=entrada.im[i][j].S;
                pir[0][i][j].I=entrada.im[i][j].I;
        }

Segmentar(umbral);
SegmentarContrasteColor(canny,umbral1,alfa,beta);

for(int i=0;i<im_seg->alto;i++)
        for(int j=0;j<im_seg->ancho;j++){

             im_seg->im[i][j].C=pir[0][i][j].C;
             im_seg->im[i][j].S=pir[0][i][j].S;
             im_seg->im[i][j].I=pir[0][i][j].I;
             im_seg->im[i][j].Clase=pir[0][i][j].Clase;
        }

}


void BIP::GenerarBaseC(int **contactos, int **contactosCanny){

register int i,j;

NV nodoAux;

for (i=0;i<ContClase;i++){
nodoAux.C=cajas[i].C;
nodoAux.S=cajas[i].S;
nodoAux.I=cajas[i].I;

nodoAux.Area=cajas[i].area;
nodoAux.padre=-1;
nodoAux.perimetro=cajas[i].perimetro;
nodoAux.hijos.clear();

for(j=0;j<ContClase;j++){
        if (contactos[i][j]!=0&&i!=j){
                nodoAux.contactos.push_back(j);
                nodoAux.num_pixel_contacto.push_back(contactos[i][j]);
                nodoAux.num_pixel_contacto_Canny.push_back(contactosCanny[i][j]);
                nodoAux.contraste.push_back(cajas[i].contraste[j]);
        }
}

NodosC[0].push_back(nodoAux);

nodoAux.contactos.clear();
nodoAux.num_pixel_contacto.clear();
nodoAux.num_pixel_contacto_Canny.clear();
nodoAux.contraste.clear();
}

}
void BIP::GenerarNivelC(int nivel,double umbral,double alfa, double beta,int **contactos, int **contactosCanny){

register int i,j,k,l;
double min;
int contacto_min;
NV nodoAux;


for(i=0;i<(int)NodosC[nivel].size();i++){

   if(NodosC[nivel][i].padre==-1){

        min=100000000.0;
        contacto_min=-1; 
        for(j=0;j<(int)NodosC[nivel][i].contactos.size();j++){

                 if((NodosC[nivel][i].contraste[j])<min){

                        min=NodosC[nivel][i].contraste[j];

                        contacto_min=NodosC[nivel][i].contactos[j];

                 }
        }
 

        if ((contacto_min!=-1) && (min<umbral)){

                if (NodosC[nivel][contacto_min].padre!=-1){
                        NodosC[nivel][i].padre=NodosC[nivel][contacto_min].padre;
                        NodosC[nivel+1][NodosC[nivel][i].padre].Area=
                                   NodosC[nivel][i].Area+NodosC[nivel][contacto_min].Area;


                        NodosC[nivel+1][NodosC[nivel][i].padre].hijos.push_back(i);
                }
                else{
                        if (NodosC[nivel][i].Area>NodosC[nivel][contacto_min].Area){
                                nodoAux.padre=-1;
                                nodoAux.Area=NodosC[nivel][i].Area+NodosC[nivel][contacto_min].Area;
                                nodoAux.contactos.clear();
                                nodoAux.contraste.clear();
                                nodoAux.hijos.clear();
                                nodoAux.num_pixel_contacto.clear();
                                nodoAux.num_pixel_contacto_Canny.clear();

                                nodoAux.C=NodosC[nivel][i].C;
                                nodoAux.S=NodosC[nivel][i].S;
                                nodoAux.I=NodosC[nivel][i].I;

                                nodoAux.perimetro=0;
                                NodosC[nivel+1].push_back(nodoAux);

                        } else{
                                nodoAux.padre=-1;
                                nodoAux.Area=NodosC[nivel][i].Area+NodosC[nivel][contacto_min].Area;
                                nodoAux.contactos.clear();
                                nodoAux.contraste.clear();
                                nodoAux.hijos.clear();
                                nodoAux.num_pixel_contacto.clear();
                                nodoAux.num_pixel_contacto_Canny.clear();

                                nodoAux.C=NodosC[nivel][contacto_min].C;
                                nodoAux.S=NodosC[nivel][contacto_min].S;
                                nodoAux.I=NodosC[nivel][contacto_min].I;

                                nodoAux.perimetro=0;
                                NodosC[nivel+1].push_back(nodoAux);
                        }
                         NodosC[nivel][i].padre= NodosC[nivel+1].size()-1;
                         NodosC[nivel][contacto_min].padre=NodosC[nivel][i].padre;
                         NodosC[nivel+1][NodosC[nivel][i].padre].hijos.push_back(i);
                         NodosC[nivel+1][NodosC[nivel][i].padre].hijos.push_back(contacto_min);
                }
        }else{

               nodoAux.C=NodosC[nivel][i].C;
               nodoAux.S=NodosC[nivel][i].S;
               nodoAux.I=NodosC[nivel][i].I;
               nodoAux.Area=NodosC[nivel][i].Area;
               nodoAux.contactos.clear();
               nodoAux.contraste.clear();
               nodoAux.hijos.clear();
               nodoAux.num_pixel_contacto.clear();
               nodoAux.num_pixel_contacto_Canny.clear();
               nodoAux.perimetro=0;
               nodoAux.padre=-1;
               NodosC[nivel+1].push_back(nodoAux);
               NodosC[nivel][i].padre=NodosC[nivel+1].size()-1;
               NodosC[nivel+1][NodosC[nivel+1].size()-1].hijos.clear();
               NodosC[nivel+1][NodosC[nivel][i].padre].hijos.push_back(i);
        }
   }
}

//Ahora voy a calcular que nodos estan en contacto

for(i=0;i<(int)NodosC[nivel].size();i++){
      for(j=0;j<(int)NodosC[nivel][i].contactos.size();j++){
         if(NodosC[nivel][i].padre!=NodosC[nivel][NodosC[nivel][i].contactos[j]].padre){
                if(std::find(NodosC[nivel+1][NodosC[nivel][i].padre].contactos.begin(),NodosC[nivel+1][NodosC[nivel][i].padre].contactos.end(),
                NodosC[nivel][NodosC[nivel][i].contactos[j]].padre)==NodosC[nivel+1][NodosC[nivel][i].padre].contactos.end())
                        NodosC[nivel+1][NodosC[nivel][i].padre].contactos.push_back(NodosC[nivel][NodosC[nivel][i].contactos[j]].padre);

                if(std::find(NodosC[nivel+1][NodosC[nivel][NodosC[nivel][i].contactos[j]].padre].contactos.begin(),NodosC[nivel+1][NodosC[nivel][NodosC[nivel][i].contactos[j]].padre].contactos.end(),
                NodosC[nivel][i].padre)==NodosC[nivel+1][NodosC[nivel][NodosC[nivel][i].contactos[j]].padre].contactos.end())
                        NodosC[nivel+1][NodosC[nivel][NodosC[nivel][i].contactos[j]].padre].contactos.push_back(NodosC[nivel][i].padre);

         }
      } //for
    }


//Ahora calculo el perimetro de los nuevos nodos


int tam=NodosC[nivel].size();
int *miradas;

miradas=new int[tam];

for(i=0; i<(int)NodosC[nivel+1].size(); i++){

    for(j=0;j<(int)NodosC[nivel].size();j++){
        miradas[j]=-1;
    }
    NodosC[nivel+1][i].perimetro=0;
    for(j=0;j<(int)NodosC[nivel+1][i].hijos.size(); j++){
            NodosC[nivel+1][i].perimetro=NodosC[nivel+1][i].perimetro+
                NodosC[nivel][NodosC[nivel+1][i].hijos[j]].perimetro;
    }

    for(j=0;j<(int)NodosC[nivel+1][i].hijos.size(); j++){
            for(k=0;k<(int)NodosC[nivel][NodosC[nivel+1][i].hijos[j]].contactos.size();k++){
                miradas[NodosC[nivel+1][i].hijos[j]]=1;

               if(NodosC[nivel][NodosC[nivel][NodosC[nivel+1][i].hijos[j]].contactos[k]].padre==i &&
                 miradas[NodosC[nivel][NodosC[nivel+1][i].hijos[j]].contactos[k]]==-1){

                   NodosC[nivel+1][i].perimetro=NodosC[nivel+1][i].perimetro-
                   NodosC[nivel][NodosC[nivel+1][i].hijos[j]].num_pixel_contacto[k];
                }
            }
    } 
}


//Ahora calculo los pixeles de contacto entre los nuevos nodos

int num_contactos;
int num_contactosCanny;

for(i=0;i<(int)NodosC[nivel+1].size();i++) {
        //NodosC[nivel+1][i].num_pixel_contacto.reserve(NodosC[nivel+1][i].contactos.size());
        //NodosC[nivel+1][i].num_pixel_contacto_Canny.reserve(NodosC[nivel+1][i].contactos.size());

        for(j=0;j<(int)NodosC[nivel+1][i].contactos.size();j++){
                num_contactos=0;
                num_contactosCanny=0;
            //NodosC[nivel+1][i].num_pixel_contacto[j]=0;
            //NodosC[nivel+1][i].num_pixel_contacto_Canny[j]=0;
            for(k=0;k<(int)NodosC[nivel+1][i].hijos.size();k++){
                for(l=0;l<(int)NodosC[nivel][NodosC[nivel+1][i].hijos[k]].contactos.size();l++){
                        if(NodosC[nivel][NodosC[nivel][NodosC[nivel+1][i].hijos[k]].contactos[l]].padre==
                           NodosC[nivel+1][i].contactos[j]){
                              //NodosC[nivel+1][i].num_pixel_contacto[j]=NodosC[nivel+1][i].num_pixel_contacto[j]+
                               // NodosC[nivel][NodosC[nivel+1][i].hijos[k]].num_pixel_contacto[l];
                              //NodosC[nivel+1][i].num_pixel_contacto_Canny[j]=NodosC[nivel+1][i].num_pixel_contacto_Canny[j]+
                               // NodosC[nivel][NodosC[nivel+1][i].hijos[k]].num_pixel_contacto_Canny[l]; //No se si es l o no
                               num_contactos=num_contactos+ NodosC[nivel][NodosC[nivel+1][i].hijos[k]].num_pixel_contacto[l];
                               num_contactosCanny=num_contactosCanny+NodosC[nivel][NodosC[nivel+1][i].hijos[k]].num_pixel_contacto_Canny[l];
                        }
                }
            }
           NodosC[nivel+1][i].num_pixel_contacto.push_back(num_contactos);
           NodosC[nivel+1][i].num_pixel_contacto_Canny.push_back(num_contactosCanny);
        }

}


//Ahora calculo el contraste nuevo
for(i=0;i<(int)NodosC[nivel+1].size();i++) {
        for(j=0;j<(int)NodosC[nivel+1][i].contactos.size();j++){
                NodosC[nivel+1][i].contraste.push_back(
                        (DisColor(NodosC[nivel+1][i],NodosC[nivel+1][NodosC[nivel+1][i].contactos[j]])*NodosC[nivel+1][i].perimetro)/
                          (alfa*NodosC[nivel+1][i].num_pixel_contacto_Canny[j]+(beta*(NodosC[nivel+1][i].num_pixel_contacto[j]-NodosC[nivel+1][i].num_pixel_contacto_Canny[j]))));
                           

        }
}

}

void BIP::ClasificarC(){
register int i,j,k;
Padre papa;

padres.clear();

ContClaseC=0;

for(k=numNivelesC-1;k>=0;k--){
 for(i=0;i<(int)NodosC[k].size();i++){
   if(NodosC[k][i].padre==-1){

                NodosC[k][i].Clase=ContClaseC;
                ContClaseC++;
                papa.Area=NodosC[k][i].Area;
                papa.posi=-1;
                papa.posj=i;
                papa.Niv=k;
                papa.NodoV=1;
                papa.C=NodosC[k][i].C;
                papa.S=NodosC[k][i].S;
                papa.I=NodosC[k][i].I;
                padres.push_back(papa);

            } else if (NodosC[k][i].padre!=-1){

                NodosC[k][i].Clase=NodosC[k+1][NodosC[k][i].padre].Clase;
            }

    }
    }

}
