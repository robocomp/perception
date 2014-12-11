#ifndef IMAGEN_H
#define IMAGEN_H


typedef  struct Pixel_Ent{
unsigned char R;
unsigned char G;
unsigned char B;

unsigned char C;
unsigned char S;
unsigned char I;

}PIXEL_ENT;

typedef  struct Pixel_Seg{
unsigned char R;
unsigned char G;
unsigned char B;

unsigned char C;
unsigned char S;
unsigned char I;

int Clase;

}PIXEL_SEG;

typedef struct Imagen_Ent{
int alto;
int ancho;
PIXEL_ENT **im;
}IMAGEN_ENT;

typedef struct Imagen_Seg{
int alto;
int ancho;
PIXEL_SEG **im;
}IMAGEN_SEG;

typedef struct Imagen_Disp{
int alto;
int ancho;
short **im;
}IMAGEN_DISP;

typedef struct Imagen_Canny{
int alto;
int ancho;
int **im;
}IMAGEN_CANNY;

typedef struct Imagen{
unsigned char **pixel;
unsigned char **C;
unsigned char **S;
unsigned char **I;
int alto;
int ancho;
short **disp;
}IMAGEN;



//void RGB_HSI(unsigned char **im,double **H, double **S, double **I, int Alto, int Ancho);
void RGB_HSI(IMAGEN_ENT *entrada);
void HSI_RGB(IMAGEN_SEG *segmentada);

//void HSI_RGB(double **H,double **S, double **I, unsigned char **R,
//             unsigned char **G, unsigned char **B, int alto, int ancho);

#endif
