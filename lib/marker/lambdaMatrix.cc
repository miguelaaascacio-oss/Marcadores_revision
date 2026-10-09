/**
	Program to calculte the lambda matrix for
	a set of points.
	The algorithm uses the Graham Scan 

	Working with indexes
	Fraga, Jul 14, 2018
**/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "marker/triangles.h"


#define MYSIZE 6   // Up to 16 points this program can calculate its lambda matrix

POINT *v0;   // The reference point
POINT *Lpts;

int comparison_index( const void *a, const void *b )
{
	int *i1, *i2;
	int n;

	i1 = (int *)a;
	i2 = (int *)b;

	n = ( Lpts[ *i1 ].x - v0->x )*( Lpts[ *i2 ].y - v0->y ) -
	    ( Lpts[ *i1 ].y - v0->y )*( Lpts[ *i2 ].x - v0->x );

	if ( n < 0 )  return 1;
	else
		return -1;
}

 
int cross_index ( int i0, int i1, int i2 )
{
	int n;

	n = ( Lpts[ i1 ].x - Lpts[ i0 ].x )*( Lpts[ i2 ].y - Lpts[ i0 ].y ) -
	        ( Lpts[ i1 ].y - Lpts[ i0 ].y )*( Lpts[ i2 ].x - Lpts[ i0 ].x );

	if ( n < 0 )  return 1;   // vectors make a right turn
	else if ( n > 0 )
		return  -1;          // vectors make a left turn
	else 
		return 0;
}


/*Computes the lambda matrix*/
void computes_LambdaMatrix( int *ind, int n, int M[][MYSIZE] )
{
    int i, j, p;
    int contadorPositivos = 0;

    for ( i = 0; i < n; i++ )
		M[i][i] = 0;

    for ( i = 0; i < n-1; i++ ) {
        for ( j = i+1; j < n; j++) {
            contadorPositivos = 0;

			// Number of points to the left of p_i and p_j

            for (p = 0; p < n; p++) {
                if (p == i || p == j) {
                    continue;
                }
				if( cross_index( ind[i], ind[j], ind[p] ) < 0 )
                    contadorPositivos++;
            }
			M[ i ][ j ] = contadorPositivos;
			M[ j ][ i ] = n - 2 - contadorPositivos;
        }
    }
}

int compare_Matrices( int A[][MYSIZE], int n, int B[][MYSIZE] )
{
	int i, j;

	for( i=0; i<n; i++ )
		for( j=i+1; j<n; j++ ) {
			if ( A[i][j] < B[i][j] )
				return -1;
			else if ( A[i][j] > B[i][j] )
				return 1;
		}

	return 0;
}

void print_LM( int A[][MYSIZE], int n )
{
	int i, j;

	for( i=0; i<n; i++ ) {
		for( j=0; j<n; j++ ) {
			if ( i == j ) 
				printf( "- " );
			else 
				printf( "%d ", A[i][j] );
		}
		printf( "\n" );
	}
}
void print_LM2( int A[][MYSIZE], int n )
{
	int i, j;

	for( i=1; i<n; i++ ) {
		for( j=i+1; j<n; j++ ) {
			// if ( i != j ) 
				printf( "%d", A[i][j] );
		}
	}
	printf("\n");
}

/**
   Input: Lpts: the list of points
   Output: indexes[]  array of integers
**/
void lambdaMatrix( POINT *LptsIN, int *indexes, int num_comp[] ) 
{
	// int do_work( char *name, int labels )
	int i, n, k, x;
	int j, j2;
	int min;
	int index0[MYSIZE];
	int index[MYSIZE];
	int indexHull[MYSIZE];
	int indexmin[MYSIZE];
	int ML0[MYSIZE][MYSIZE];
	int ML1[MYSIZE][MYSIZE];
	int bandera = 0;

	n = MYSIZE;
	Lpts = LptsIN;

	/** find the point with minimum y-coordinate **/
	min = 0;
	for( i=1; i<n; i++ ) 
		if ( Lpts[i].y < Lpts[min].y )
			min = i;

	/** among all such points, find the one with minimum
	    x-coordinate  **/
	for( i=0; i<n; i++ ) 
		if ( ( Lpts[i].y == Lpts[min].y) && ( Lpts[i].x < Lpts[min].x) )
			min = i; 

	// printf( "min = %d\n", min );

	v0 = &Lpts[min];	

	index[0] = min;
	k = 1;
	for( i=0; i<n; i++ ) {
		if( i != min ) {
			index[k] = i;
			k++;
		}
	}

	qsort( (void *)&index[1], n-1, sizeof(int), comparison_index );
	for( i=0; i<n; i++ ){ 
		indexHull[i] = index[i];
		indexmin[i] = index[i];
	}

	/** Lambda matrix with indexes to the readed points **/ 
	computes_LambdaMatrix( indexHull, n, ML0 ); 

	/** Discard points not on the convex hull **/
	k = 1;
	for ( i=2; i<n; i++ ) { 
		while ( cross_index( indexHull[k-1], indexHull[k], indexHull[i] ) > 0 && k>0) 
			k--;  
		k++;
		if ( k != i ) {
			// Swap indexes
			x = indexHull[i];
			indexHull[i] = indexHull[k];
			indexHull[k] = x;
		} 
	}

	
	/**
	printf( "# Convex hull points %d\n", k+1 );
	printf( "# i  x  y\n" );
	for ( i=0; i<=k; i++ ) 
		printf ( "%d %d\n", Lpts[ indexHull[i] ].x, Lpts[ indexHull[i] ].y );
	**/

	/** Now there are k+1 points in the convex hull

	Hay k+1 matrices lambda, cada una calculada
	con respecto a cada uno de los k+1 puntos en la cubierta convexa.

	Problema: Para calcular las matrices lambda se debe realizar:

	1) El primer punto calculado ya está en la cubierta convexa.
	   Este punto es el mínimo en y, y si hay varios con y minima,
	   es el del mínimo x. 

	2) En este programa los índices de los puntos están en la secuencia
	   en que fueron leídos.

	3) Los índices a los puntos ya están ordenados con respecto al índice de este primer punto
		( están en el arreglo index[] )

	4) Hay que calcular los puntos a la izquierda de cada par de líneas
	   ( por cada par de puntos en index[] ).

	Matrix lambda

	  0  1  2
	0 -
	1    -
	2       -

	Hay tres puntos. n = 3
	Hay que calcular
	lambda( 0, 1 ) = v1
	lambda( 0, 2 ) = v2
	lambda( 1, 2 ) = v3

	Los demás valores serán:
	lambda( 1, 0 ) = n - 2 - lambda( 0, 1 )
	lambda( 2, 0 ) = n - 2 - lambda( 0, 2 )
	lambda( 2, 1 ) = n - 2 - lambda( 1, 2 )

	lambda( 2, 1 ) + lambda( 1, 2 ) = n - 2


	5) Para las demás matrices lambda hay que ordenar los índices con respecto
	  al resto de los puntos en la cubierta convexa
	  Los índices a los puntos en la cubierta convexa están en el arreglo
	  indexHull[]

	6) Ya no hay que repetir el paso (4) !!!
	  Dado que los índices (el etiquetado) se mantiene, solo cambia
	  el orden de los puntos en el arreglo ordenado index[]
    **/

	// Form the rest of k lambda matrices
	// Keep the new one if it is lesser than the previos one
	bandera = 0;
	for( j=0; j<n; j++ )
		indexmin[j] = index[j];

	for( j=0; j<n; j++ )
		index0[ indexmin[j] ] = j;

	for( i=1; i<=k; i++ ) {
		v0 = &Lpts[ indexHull[ i ] ];	
		index[0] = indexHull[ i ];

		j = 1;
		for( j2=0; j2<n; j2++ ) {
			if( j2 != indexHull[ i ] ) {
				index[j] = j2;
				j++;
			}
		}
		qsort( (void *)&index[1], n-1, sizeof(int), comparison_index );
		/**
		printf( "Orden:\n" );
		for( j=0; j<n; j++ ) 
			printf( "%d\n", index[j] );
		**/
		// The new Lambda matrix
		for( j=0; j<n-1; j++ ) {
			for( j2=j+1; j2<n; j2++ ) {
				ML1[ j ][ j2 ] = ML0[ index0[ index[j] ] ][ index0[ index[j2]] ];  
				ML1[ j2 ][ j ] = ML0[ index0[ index[j2]] ][ index0[ index[j] ] ];  
			}
		}
		// print_LM( ML1, n );
	
		// Compare both matrices!
		j2 = compare_Matrices( ML0, n, ML1 );
		if( j2  > 0 ) { // ML1 < ML0
			for( j=0; j<n; j++ )
				for( j2=0; j2<n; j2++ )
						ML0[ j ][ j2 ] = ML1[ j ][ j2 ];

			for( j=0; j<n; j++ )
				indexmin[j] = index[j];

			for( j=0; j<n; j++ )
				index0[ indexmin[j] ] = j;
		}
		else if ( j2 == 0 )
			bandera = 1;
	
	}

	for( j=0; j<n; j++ ){
		indexes[j] = indexmin[j];
		//printf("%d",indexes[j]);
	}

	
	//print_LM2( ML0, n );

	num_comp[0]= ML0[1][5];  
	num_comp[1]= ML0[2][3];
	//printf("num_comp 0: %d ",num_comp[0]);
	//printf("num_comp 1: %d ",num_comp[1]);
	// printf( " \n" );
	// if ( bandera ){
	// 	print_LM2( ML0, n );
	// 	printf( "\n " );
	// }

	

}

