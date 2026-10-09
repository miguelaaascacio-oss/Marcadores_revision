// ----------------------------------------------------------------------------
// Numerical diagonalization of 3x3 matrcies
// Copyright (C) 2006  Joachim Kopp
// ----------------------------------------------------------------------------
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
// ----------------------------------------------------------------------------
// #include "dsyevc3.h"
// #include "dsyevv3.h"


// Macros
#define SQR(x)      ((x)*(x))                        // x^2 

double cubic_root( double v ); 

/** 
 Args: ax^3 + bx^2 + cx + d = 0
 double * roots must be pointer to an array of size three 
 Returns the number of real root and their values insise roots[]
**/
int cubic( double a, double b, double c, double d, double *roots );

/**
  Calcula los eugenvalores de una matriz simétrica positiva definida A
**/
int eigenValues( double A[][3], double *roots );


// ----------------------------------------------------------------------------
int dsyevv3(double A[3][3], double Q[3][3], double w[3]);

