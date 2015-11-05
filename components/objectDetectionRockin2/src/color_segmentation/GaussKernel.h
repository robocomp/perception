#include <iostream>

using namespace std;

/* GaussianDiscrete2D: Calcula el valor discreto en x,y de la
distribuci�n gaussiana en 2D.
Valores de entrada: theta:valor de theta para la distribuci�n
                        gaussiana.
                    x,y:puntos a los que calcular el valor discreto.
Salida: valor gaussiano discreto. */

double GaussianDiscrete2D(double theta, int x, int y);

/* Gaussian 2D: Calcula algunos valores discretos de la distribuci�n
gaussiana en 2D.
Entradas: theta:valor de theta para la distribuci�n
                        gaussiana.
          size: n�mero de valores discretos(p�xeles) a calcular.
Salida: kernel:valores calculados.*/

void Gaussian2D(double **kernel, double theta,int size);

/* Smooth: Coge una imagen y una distribuci�n gaussiana, calcula el kernel �
apropiado y lo aplica a la imagen para suavizarla*/

void Smooth(double **im, double **ims,int width,int height, int size, double theta);
void SmoothImage(int **im,int **dest,int w,int h,int size,double theta);

 
