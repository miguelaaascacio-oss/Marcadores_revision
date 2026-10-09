#include <stdio.h>
#include "marker/triangles.h"
#include "opencv2/core/mat.hpp"

using namespace cv;

// Sigo con la convención (y,x)
static int MEvoy[10][2]={{0,0}, {-1,0}, {-1,1}, {0,1}, {1,1}, {1,0}, {1,-1}, {0,-1}, {-1,-1}, {-1,0} }; 
// (-1,-1  (-1,0) (-1,1)     8  1  2
// ( 0,-1) ( y,x) ( 0,1)     7  0  3
// ( 1,-1) ( 1,0) ( 1,1)     6  5  4


int signedArea( int p1, int p2 )
{
	return( MEvoy[p1][1]*MEvoy[p2][0] - MEvoy[p1][0]*MEvoy[p2][1] ); 
}

int follow_perimeter( Mat& I, int y, int x, uchar tono, POINT *pts  )
{
	int i, j, k, P[9], Porden[10];
	int vecinos[9];
	int cuenta=0;
	int tmp;
	int *ptri1, *ptri2, sp;
	
	uchar *inptr, *pimg;

	do {
		// printf( "Per: %d %d\n", x, y );

		// El punto inicial del perímetro es y, x
		for ( k=0, i = -1 ; i <= 1; i++ ) {
			inptr = I.ptr<uchar>( y+i );
			inptr = inptr + x - 1;
			for ( j = 0 ; j < 3; j++ ){
				P[k++] =  ( *inptr++ == tono ) ? 1 : 0;
			}
		}

		Porden[0] = P[4]; Porden[1] = P[1]; Porden[2] = P[2];
		Porden[3] = P[5]; Porden[4] = P[8]; Porden[5] = P[7];
		Porden[6] = P[6]; Porden[7] = P[3]; Porden[8] = P[0];
		Porden[9] = P[1];

		/** No. de transiciones de 0-1 **/
		ptri1 = Porden;
		ptri2 = ptri1 + 1;
		sp = 0;
		for( i=0; i<=8; i++ ) {
			if( *ptri1==0 && *ptri2>0 ){
				vecinos[sp++] = i+1;
			}
			ptri1++;
			ptri2++;
		} 

		// printf( "%d vecinos\n", k );

		// El punto (y,x) tiene k vecinos
		// Si k=1, borramos (y,x) y nos vamos a ese vecino
		// Si k=2, los ordenamos, borramos (y,x) y nos vamos al siguiente punto
		// Si k=3, los ordenamos, borramos (y,x) y nos vamos al siguiente punto
		// Si k>=4, es un error?
		if ( sp==0 ) {
			// printf("%d %d\n", x, y );
			pimg = I.ptr<uchar>( y ) + x;
			*pimg = 0;
			pts[cuenta].x = x;
			pts[cuenta].y = y;
			cuenta++;
			break;  // Ya no hay puntos del perímetro, salimos del ciclo do-while
		}
		else if ( sp==1 ) {
			goto AQUI;
		}
		else if ( sp==2 ) {
			// Ordeno los puntos (y,x), vecinos[1] y vecinos[2]
			// printf( "%d %d\n", vecinos[0], vecinos[1] );
			if ( signedArea( vecinos[0], vecinos[1] ) < 0 ) {
				tmp = vecinos[0];
				vecinos[0] = vecinos[1];
				vecinos[1] = tmp;
			}
		}
		else {
			// printf("%d %d\n", x, y );
			// fprintf( stderr, "Esto ocurre en un perímetro???\n" );
			return -1;
		}
		AQUI:
		// Borro el pixel (y,x)
		pimg = I.ptr<uchar>( y ) + x;
		*pimg = 0;	
		// printf("%d %d\n", x, y );
		pts[cuenta].x = x;
		pts[cuenta].y = y;
		cuenta++;
		y += MEvoy[ vecinos[0] ][0];
		x += MEvoy[ vecinos[0] ][1];
	} while( 1 );
	
	return cuenta;
}
