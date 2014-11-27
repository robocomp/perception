#include <iostream>

using namespace std;

/* GaussianDiscrete2D: Calcula el valor discreto en x,y de la
distribución gaussiana en 2D.
Valores de entrada: theta:valor de theta para la distribución
                        gaussiana.
                    x,y:puntos a los que calcular el valor discreto.
Salida: valor gaussiano discreto. */

double GaussianDiscrete2D(double theta, int x, int y);

/* Gaussian 2D: Calcula algunos valores discretos de la distribución
gaussiana en 2D.
Entradas: theta:valor de theta para la distribución
                        gaussiana.
          size: número de valores discretos(píxeles) a calcular.
Salida: kernel:valores calculados.*/

void Gaussian2D(double **kernel, double theta,int size);

/* Smooth: Coge una imagen y una distribución gaussiana, calcula el kernel ç
apropiado y lo aplica a la imagen para suavizarla*/

void Smooth(double **im, double **ims,int width,int height, int size, double theta);
void SmoothImage(int **im,int **dest,int w,int h,int size,double theta);

 
