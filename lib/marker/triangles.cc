#include <math.h>
#include <float.h>
#include "marker/follow.h"
#include "marker/triangles.h"

extern int checa_linea( POINT * pts, int, int );


int getVector( POINT *p, int i1, int i2, DPOINT *pmedia, DPOINT *vec )
{
	int i,  n;
	double mediax, mediay;
	double x, y, a, b, c;
	double mat[4];
	double v, vmin, vmax;
	double epsilon;

	if ( i2 <= i1 )
		return -1;

	// printf( "i's: %d %d\n", i1, i2 ) ;

	mediax = mediay = 0.0;
	for( i = i1; i <= i2; i++ ){
		mediax += p[i].x;
		mediay += p[i].y;
	}
	n = i2-i1+1;
	mediax /= n;
	mediay /= n;

	pmedia->x = mediax;
	pmedia->y = mediay;

	a = b = c = 0.0;
	for ( i = i1; i<= i2; i++){
		x = p[i].x - mediax;
		y = p[i].y - mediay;
		a += x*x;
		b += x*y;
		c += y*y;
	}
	// La matriz es ahora
	// [ a b ]
	// [ b c ]

	// Calculamos los eigenvalores
	x = a - c;
	v = x * x + 4.0 * b*b;
	v = sqrt(v);
	vmin = ((c + a) - v) / 2.0;
	vmax = ((c + a) + v) / 2.0;
	mat[0] = a - vmin;
	mat[1] = b;
	mat[2] = b;
	mat[3] = c - vmin;

	v = mat[0] * mat[0] + mat[1] * mat[1];
	epsilon = 8.0 * DBL_EPSILON * vmax;

	if (v <= epsilon) {
		vec[0].x = 0.0;
		vec[0].y = 1.0;
	}
	else {
		v = mat[2] * mat[2] + mat[3] * mat[3];

		if (v <= epsilon) {
			vec->x = 1.0;
			vec->y = 0.0;
		}
		else {
			v = sqrt(v);
			vec->x = mat[1] / v;
			vec->y = mat[3] / v;
		}
	}
	return 0;
}

int intersecs( DPOINT *p1, DPOINT *v1, DPOINT *p2, DPOINT *v2, DPOINT *r )
{
	double delta, deltaS, deltaT;
	double px, py;
	double T, S;

	//Compute the intersection of edge1 with edge2.
	delta	= -( v1->x * v2->y ) + v1->y * v2->x;
	if( fabs(delta) < 1e-15 )
		return -1;

	px = p2->x - p1->x;
	py = p2->y - p1->y;

	deltaS  = v1->x * py - v1->y * px;
	deltaT  = py * v2->x  - v2->y * px;

	T = deltaT/delta;
	S = deltaS/delta;

	r->x = p1->x + T*v1->x;
	r->y = p1->y + T*v1->y;

	return 0;
}

int extrae_triangulo( POINT *pts, int n, POINT vertices[4], DPOINT dv[3] )
{
	int i;
	int currentDistance, maxDistance, maxPosition1, maxPosition2, xtmp, ytmp;
	int abx,aby,acx,acy,bcx,bcy;
	float dtmp_3, d3;
	float e,f;
	DPOINT media1, media2, media3;
	DPOINT vec1, vec2, vec3;

	for( i=0; i<3; i++ ) {
		vertices[i].x = -1;
		vertices[i].y = -1;
	}

	//At this point array pts should be filled with n coordinates values.
	//Here we find the three vertices by going two times around each array.
	//By definition our first position is the first vertex, so we save it as such.
	vertices[0].x = pts[0].x;
	vertices[0].y = pts[0].y;

	currentDistance = 0;
	maxDistance = 0;
	maxPosition1 = 0; //Current max is the first vertex.
	maxPosition2 = 0;
// printf(">> DEBUG: 1\n");
	//Second vertex is the vertex furthest away from the first vertex.
	for ( i = 1; i < n; i++){
		xtmp = vertices[0].x - pts[i].x;
		ytmp = vertices[0].y - pts[i].y;
		currentDistance = xtmp*xtmp + ytmp*ytmp;
		if (currentDistance > maxDistance){
			//We have found a point that is furthest away than previous point...
			//Let us update vertex 2 and the actual position of it on the list of points.
			maxDistance = currentDistance;
			maxPosition1 = i;
		}
	}
// printf(" >> DEBUG: 2\n");
	vertices[1].x = pts[maxPosition1].x;
	vertices[1].y = pts[maxPosition1].y;

	//At this point we have obtained second vertex and its position on the list.
	// printf("# Second point...%d,%d\n", vertices[1].x, vertices[1].y );
	// printf("#Position...%d\n",maxPosition);
	// printf("#Listing first %d vertices...\n",maxPosition);
	// for ( i = 0; i < maxPosition; i++){
	//	 printf("%d %d\n",puntosx[i],puntosy[i]);
	//}
	//From the point at maxPosition until nPoints we check the distance
	//from each point to the line segment created by (vx[0],vy[0]) - (vx[1],vy[1])
	//Then we take the point that maximizes this distance as our third vertex.
	//taken from collision detection book
	acx = 0;
	bcx = 0;
	acy = 0;
	bcy = 0;
	e = 0;
	f = 0;
	dtmp_3 = 0;
	xtmp = 0;
	ytmp = 0;
	d3 = 0;

	abx = vertices[1].x - vertices[0].x;
	aby = vertices[1].y - vertices[0].y;

	for ( i = 1; i < n; i++) {
		//a y b son el primer y segundo vértice
		//c es el punto actual.
		acx = pts[i].x - vertices[0].x; acy = pts[i].y - vertices[0].y;
		bcx = pts[i].x - vertices[1].x; bcy = pts[i].y - vertices[1].y;
		//ac*ab
		e = acx*abx + acy * aby;
		if (e<=0) dtmp_3 = acx*acx + acy*acy;
		f = abx*abx + aby*aby;
		if (e >= f) dtmp_3 = bcx*bcx + bcy*bcy;
		else dtmp_3 = acx*acx + acy*acy - e*e / f;
		if (dtmp_3 > d3){
			d3 = dtmp_3;
			maxPosition2 = i;
		}
	}
	vertices[2].x = pts[maxPosition2].x;
	vertices[2].y = pts[maxPosition2].y;

	if ( maxPosition2 < maxPosition1 ) {
		xtmp = maxPosition1;
		maxPosition1 = maxPosition2;
		maxPosition2 = xtmp;

		vertices[1].x = pts[maxPosition1].x;
		vertices[1].y = pts[maxPosition1].y;
		vertices[2].x = pts[maxPosition2].x;
		vertices[2].y = pts[maxPosition2].y;
	}

	/**
	At this point we have separated the sets of 2 of the edges of the triangle...
	First edge is from index 0 to maxPosition1,
	Second edge is from maxPosition1 to maxPosition2...
	Third edge is clearly from maxPosition2 to n-1.
	We must then feed these three sets of points to the function that
	given a set of points, returns a ray (point + vector).
	Finally, we'll have 3 rays, and the last step is to calculate the pairwise
	intersection of consecutive rays (there will be 3).
	**/

	/**
	if ( checa_linea( pts, 0, maxPosition1-1 ) >= 0 ){
		printf("# t e1\n");
		return -4;
	}
	if ( checa_linea( pts, maxPosition1, maxPosition2-1 ) >= 0 ){
		printf("# t e2\n");
		return -5;
	}
	if ( checa_linea( pts, maxPosition2, n-1 ) >= 0 ){
		printf("# t e3\n");
		return -6;
	}
	**/

	//Isolate each set of points for each edge.
	//Edge 1.
	if( getVector( pts, 0, maxPosition1-1, &media1, &vec1 ) == 0 ){ 
		// printf("Vector 1: %lf %lf %lf %lf\n", media1.x, media1.y, vec1.x, vec1.y );
	}
	else 
		return -1;

	//Edge2.
	if ( getVector( pts, maxPosition1, maxPosition2-1, &media2, &vec2 ) == 0 ){
		// printf("Vector 2: %lf %lf %lf %lf\n", media2.x, media2.y, vec2.x, vec2.y );
	}
	else 
		return -1;

	//Edge3.
	if( getVector( pts, maxPosition2, n-1, &media3, &vec3 ) == 0 ){
		// printf("Vector 3: %lf %lf %lf %lf\n", media3.x, media3.y, vec3.x, vec3.y );
	}
	else 
		return -1;

	//Compute the intersection of edge1 with edge2.
	intersecs( &media3, &vec3, &media1, &vec1, &dv[0] );
	if( dv[0].x < 0.0 || dv[0].x > 640.0 )
		return -1;
	if( dv[0].y < 0.0 || dv[0].y > 480.0 )
		return -1;
	intersecs( &media1, &vec1, &media2, &vec2, &dv[1] );
	if( dv[1].x < 0.0 || dv[1].x > 640.0 )
		return -1;
	if( dv[1].y < 0.0 || dv[1].y > 480.0 )
		return -1;
	intersecs( &media2, &vec2, &media3, &vec3, &dv[2] );
	if( dv[2].x < 0.0 || dv[2].x > 640.0 )
		return -1;
	if( dv[2].y < 0.0 || dv[2].y > 480.0 )
		return -1;

	return 0;
}

