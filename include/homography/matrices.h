/*
 * Contiene la implementación de operaciones de matrices, 
 * como multiplicación, copia, cálculo de norma de vectores
 * así como la descomposición QR.
 */


#include <stdio.h>
#include <math.h>

#ifndef MATRICES
#define MATRICES
/* 
   Realiza C = A * B
*/
void multiplicaAB(double A[][3], double B[][3], double C[][3]);

/* 
   Realiza C = A^T * B
*/
void multiplicaATB(double A[][3], double B[][3], double C[][3]);

/* 
   Realiza C = A * B^T
*/
void multiplicaABT(double A[][3], double B[][3], double C[][3]);

/*
 * Calcula la transpuesta de una matriz
 */ 
void TranspuestaMatriz(double A[3][3], double B[3][3]);

/*
 * Multiplica la Matriz A por el escalar a 
 **/
void Mult_Escalar(double a, double A[3][3], double A2[3][3]);

// B = A
void CopiaMatriz(double A[3][3], double B[3][3]);

/*
 * Vector columna 
 * */
void Mult_Mat_Vector(double A[3][3],  double vb[3], double vc[3]);

/*Imprime Vector Columna */
void ImprimirVector(double va[3]);

/*Imprime la matriz 
 **/
void imprime(double A[3][3]);

/*Norma de tres valores */
double Norma(double a, double b, double c);

/*Norma del vector */
double norma (double *vec,  int n );

void QR(double A[8][8], double Q[8][8], double R[8][8] );

void QR_noQ( double A[][9], double R[][9] );

#endif
