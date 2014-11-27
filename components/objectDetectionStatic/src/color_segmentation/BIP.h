#ifndef BIP_H
#define BIP_H
#include <vector>
#include "Canny.h"
#include "Imagen.h"
#include <math.h>
#include <cmath>


#define AREAMIN 20

typedef struct Punto{
 int i;
 int j;
}PUNTO;


typedef struct Piramide{
	int Homog;
	int X;
	int Y;
	int Area;
	int Clase;
	int ClaseObj;
	int primero;
	std::vector<int> contactosV;
	std::vector<PUNTO> contactosR; //Para los casos especiales en los que dos nodos regulares
							 //estan en contacto y no son vecinos espacialmente

	unsigned char contrasteTotal;
	unsigned char contrasteTotalI;
	unsigned char saliencyMap;
	short disp;
	unsigned char C;
	unsigned char S;
	unsigned char I;

	unsigned char Cor;
	unsigned char Sor;
	unsigned char Ior;

}PIR;

/*Padre: estructura para almacenar las características de los
nodos padres de la estructura.
posi,posj - posición del nodo padre, si es un nodo basico de la estructura.
Niv - nivel del nodo padre.
NodoV - si el padre es un nodo virtual de primer orden, aquí se almacena
su posición en el array de nodos virtuales de primer orden.
H,S,I - valores H, S e I del nodo padre.
Area - area de la region que corresponde al nodo en la base.*/

typedef struct padre{
	double posi;
	double posj;
	int Niv;
	int NodoV;
	int Area;
	double contrasteTotal;
	double contrasteTotalI;
	double saliencyMap;
	short disp;
	int C;
	int S;
	int I;
}Padre;

/*NV: Estructura para almacenar las características de los nodos virtuales.
H,S,I - valores H, S, e I del nodo.
Clase - clase a la que pertenece el nodo.
Area - area de la region que corresponde al nodo en la base.*/

typedef struct nv{
	int Clase;
	int Area;
	int padre;
	std::vector<int> contactos;
	std::vector<PUNTO> contactosR;
    	std::vector<double> contraste;
	std::vector<int> num_pixel_contacto;
	std::vector<int> num_pixel_contacto_Canny;
	int perimetro;
	std::vector<int> hijos;
	short disp;
	unsigned char C;
	unsigned char S;
	unsigned char I;
}NV;

/*DIST: Estructura que amacena la distancia en color entre dos nodos .
dif - distancia en color.
x,y - coordenadas de uno de los nodos que se están comparando.*/

typedef struct Dist{
       double dif;
       int x;
       int y;
}DIST;

/* BOX: Estructura para almacenar las bounding box de las clases.
imin,imax,jmin,jmax- coordenadas de los vertices de la caja.
H,S,I- valores H,S e I de la clase encerrada en la caja.
area- area de la clase.
icent,jcent- coordenadas del centroide de la caja
areanivel-  area en ese nivel
encontrada- marca si la region encerrada en la caja ha sido ya encontrada en
el template matching*/

typedef struct Box{
 int imin;
 int imax;
 int jmin;
 int jmax;
long double i1,i2,i3,i4,j1,j2,j3,j4;
 int area;
 int areanivel;
 int sumi;
 int sumj;
 bool encontrada;
long double icent;
 long double jcent;
 bool en_estereo;

 double *contraste;
 double perimetro;

 double teta;
 short disp;
  int C;
  int S;
  int I;

}BOX;

typedef struct Fus{
 int clase;
 int C;
 int S;
 int I;
 int Area;
}FUS;


class CmpContactos{
PUNTO p;
public:
CmpContactos(PUNTO &_p):p(_p){}
bool operator()(const PUNTO p1){
                return ((p1.i==p.i)&&(p1.j==p.j));
                }
};




/****************************************************************************

                DEFINICI�N DE LA CLASE BIP

****************************************************************************/



class BIP{
public:
        int niveles;
        int *TamNivelesAncho;
        int *TamNivelesAlto;
        std::vector<NV> *NodosV;
        PIR ***pir;
        int altoim;
        int anchoim;
        double umbralS;
        double umbralI;
        double umbralColor1;
        int ContClase;
        std::vector<Padre> padres;
        double *umbralColor;
        int nivelesI;
        int nivelesC;
        int numNiveles;
        BOX *cajas;
        BOX *cajasmin;
        std::vector<NV> *NodosC;
        int ContClaseC;
        int numNivelesC;
	// std::vector<MARCAS> marcas;

        BIP(int _altoim, int _anchoim, int nivelesI, double umbral);
        ~BIP();
        void Segmentar(double umbral);
        void SegmentarContrasteColor(IMAGEN_CANNY canny,double umbral,double alfa, double beta);
        void PerceptualSegmentation(IMAGEN_ENT entrada,double umbral,double umbral1, IMAGEN_CANNY canny,double alfa, double beta, IMAGEN_SEG *im_seg);


private:
        int GenerarParteRegular(int nivel);
        void GenerarParteIrregular(int nivel,double umbral);
        void Clasificar();
	void LLevarColoresArriba();
        void FusionAreasPequenas();
        void GenerarBaseC(int **contactos, int **contactosCanny);
        void GenerarNivelC(int nivel,double umbral,double alfa, double beta,int **contactos, int **contactosCanny);
        void ClasificarC();


};


template<typename color, typename color1> double DisColor(color nodo1, color1 nodo2){

  double d;
  double di,dc;
  double H1,H2;
  double teta;


H1=nodo1.C*360/255;
H2=nodo2.C*360/255;

if (fabs((H1)-(H2))<180) teta=fabs((H1)-(H2));
else teta=360-(fabs((H1)-(H2)));

teta=(teta*3.1415)/180;

di= (double)std::abs(nodo1.I-nodo2.I);
dc= sqrt((double)(nodo1.S*nodo1.S)+(double)(nodo2.S*nodo2.S)-
          (2*nodo1.S*nodo2.S*cos(teta)));

d=sqrt((di*di)+(dc*dc));

return d;

}

template <typename celda, typename celda1> NV CrearNodoVirtual(celda nodo1, celda1 nodo2){

PUNTO p1,p2,medio;
double H,teta;
int area;
NV nodoV;

//Extrae las componentes vectoriales del color de los dos nodos

teta=nodo1.C*2*3.1415/255;
p1.i=nodo1.S*cos(teta);
p1.j=(nodo1.S*sin(teta));

teta=nodo2.C*2*3.1415/255;
p2.i=nodo2.S*cos(teta);
p2.j=nodo2.S*sin(teta);

area=nodo1.Area+nodo2.Area;
//Calcula el valor medio de los dos vectores de color

medio.i=(double)((double)((p1.i*nodo1.Area)+(p2.i*nodo2.Area))/(double)(area));
medio.j=(double)((double)((p1.j*nodo1.Area)+(p2.j*nodo2.Area))/(double)(area));

 if(medio.j==0 && medio.i==0) H=0;
 else H=(atan2(medio.j,medio.i))*180/3.1415;

if(H<0) H=360+H;
H=H*255/360;

//Asigna la media de color entre los dos nodos al nodo virtual padre

nodoV.I=(unsigned char)(((nodo1.I*nodo1.Area)+(nodo2.I*nodo2.Area))/(area));
nodoV.C=(unsigned char)H;
nodoV.S=(unsigned char)sqrt((medio.i*medio.i)+(medio.j*medio.j));

//Calcula el �rea del nodo virtual

nodoV.Area=area;
nodoV.Clase=-1;
//nodoV.ClaseObj=-1;
nodoV.padre=-1;

return nodoV;

}


#endif
