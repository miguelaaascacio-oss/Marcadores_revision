/*
 * Contiene la implementación de operaciones de matrices, 
 * como multiplicación, copia, cálculo de norma de vectores
 * así como la descomposición QR.
 */
#include "homography/matrices.h"

#include <stdio.h>
#include <math.h>

#define N 6

/* 
   Realiza C = A * B
*/
void multiplicaAB(double A[][3], double B[][3], double C[][3])
{
	int i, j;
	for (i=0; i<3;i++)
		for (j=0;j<3;j++)
			C[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j];
}

/* 
   Realiza C = A^T * B
*/
void multiplicaATB(double A[][3], double B[][3], double C[][3])
{
	int i, j;
	for (i=0; i<3;i++)
		for (j=0;j<3;j++)
			C[i][j] = A[0][i]*B[0][j] + A[1][i]*B[1][j] + A[2][i]*B[2][j];
}

/* 
   Realiza C = A * B^T
*/
void multiplicaABT(double A[][3], double B[][3], double C[][3])
{
	int i, j;
	for (i=0; i<3;i++)
		for (j=0;j<3;j++)
			C[i][j] = A[i][0]*B[j][0] + A[i][1]*B[j][1] + A[i][2]*B[j][2];
}

/*
 * Calcula la transpuesta de una matriz
 */ 
void TranspuestaMatriz(double A[3][3], double B[3][3]){
  int i, j;
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      B[i][j] = A[j][i];
    }
  }
}

/*
 * Multiplica la Matriz A por el escalar a 
 **/
void Mult_Escalar(double a, double A[3][3], double A2[3][3]){
  int i,j;
  for (i = 0; i < 3; i++) {
    for (j = 0; j <3; j++) {
      A2[i][j] = A[i][j] * a;
    }
  }
}


// B = A
void CopiaMatriz(double A[3][3], double B[3][3]) {
  int i, j;
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      B[i][j] = A[i][j];
    }
  }
}

/*
 * Vector columna 
 * */
void Mult_Mat_Vector(double A[3][3],  double vb[3], double vc[3])
{
  int i,j;

  for (i = 0; i < 3; i++) {
       vc[i] = 0.0;
    for (j = 0; j < 3; j++) {
       vc[i] += A[i][j]*vb[j];
    }
  }
}

/*Imprime Vector Columna */
void ImprimirVector(double A[3])
{
  int i;
  for (i = 0; i < 3; i++) {
    printf("%f\n", A[i] );
  }
  printf("\n");
}

/*Imprime la matriz 
 **/
void imprime(double A[3][3])
{
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            printf("%f\t", A[i][j]);
        }
        printf("\n");
    }
}


/*Norma de tres valores */
double Norma(double a, double b, double c)
{
  return( sqrt(a*a + b*b + c*c) );
}

/*Norma del vector */
double norma (double *vec,  int n )
{
    double acum;
    int i;

    acum = 0;
    for (i=0; i < n; i++){
        acum += vec[i]*vec[i];
    }
    return sqrt(acum);
}


void QR(double A[8][8], double Q[8][8], double R[8][8] )
{
    double V[8][8];
    double vec[8];
    int i, j, k;
    //copia
    for (i = 0; i < 8; i++)
        for (k = 0; k < 8; k++){
            V[i][k] = A[i][k];
            Q[i][k] = R[i][k] = 0;
        }
    //algoritmo
    for (i = 0; i < 8; i++){
        for (k = 0; k < 8; k++) vec[k] = V[k][i];
        R[i][i] = norma(vec, 8);
        for (k = 0; k < 8; k++) 
            Q[k][i] = V[k][i] / R[i][i]; 
        for (j = i+1; j < 8; j++){
            R[i][j] = 0;
            for (k = 0; k < 8; k++) R[i][j] += Q[k][i] * V[k][j];
            for (k = 0; k < 8; k++) V[k][j] = V[k][j] - R[i][j] * Q[k][i];
        }
    }
}

/**
  Subrutina que calcula descomposición QR
  A = QR, A es de tamaño N*2 x 9
  No retorna el valor de  Q
**/
void QR_noQ( double A[N*2][9], double R[N*2][9] )
{
	int i, j, k;
	double sum, tmp;
	int m=N*2;
	int n=9;
	double V[N*2][9], vec[N*2];

    for (i = 0; i < m; i++)
        for (k = 0; k < n; k++){
            V[i][k] = A[i][k];
        }

	for( k=0; k<n; k++ ) {

		 sum = 0.0;
		 for( i=0; i<m; i++ ) {
			tmp = V[i][k];
			sum += tmp * tmp;
		 }
		 sum = sqrt( sum );
		 R[k][k] = sum;
		 for( i=0; i<m; i++ )
			vec[i] = V[i][k] / sum;

		 for( j=k+1; j<n; j++ ) {
			sum = 0.0;
			for( i=0; i<m; i++ )
				sum += vec[i] * V[i][j];

				R[k][j] = sum;

			for( i=0; i<m; i++ )
				V[i][j] -= vec[i] * sum;
		 }
	}
}

