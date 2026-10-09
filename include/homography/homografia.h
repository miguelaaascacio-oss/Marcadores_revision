
/*
 * calcula las funciones necesarias para calcular las matrices
 * K  R y t que del modelo de la cámara obscura, dadas las esquinas de
 * un cuadrado, indicadas en una imágen.
 * */
#define N 6

void normalizar(double S[][2], float* E, double vd[], int n);
void CalculaR(double A[][3], double R[][3]);
int parametros_de_H(double H[3][3], float MK[16], float MRT[16], double Hi,
                    double Wi);
int homografia(double Pnor[][2], double vdP[4], float* p, float MK[16], float MRT[16],
               double height, double width);
