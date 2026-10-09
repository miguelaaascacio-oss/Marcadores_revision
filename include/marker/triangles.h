#ifndef TRIANGLES_H
#define TRIANGLES_H 1

typedef struct point {
    int x, y;
} POINT;

typedef struct dpoint {
    double x, y;
} DPOINT;

int extrae_triangulo( POINT *pts, int n, POINT *vertices, DPOINT *dv );
int getVector( POINT *p, int i1, int i2, DPOINT *pmedia, DPOINT *vec );
int intersecs( DPOINT *p1, DPOINT *v1, DPOINT *p2, DPOINT *v2, DPOINT *r );

#endif
