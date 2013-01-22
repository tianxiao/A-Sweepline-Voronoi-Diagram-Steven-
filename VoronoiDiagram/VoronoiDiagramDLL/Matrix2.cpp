#include "StdAfx.h"
#include <assert.h>
#include <math.h>
#include "Matrix2.h"

#define MATRIX_DETERMINE_PRECISION 1e-15

txMatrix2::txMatrix2(void)
{
	m[0][0] = 0.0;
	m[0][1] = 0.0;
	m[1][0] = 0.0;
	m[1][1] = 0.0;
}

txMatrix2::txMatrix2(double a00, double a01, double a10, double a11) {
	m[0][0] = a00;
	m[0][1] = a01;
	m[1][0] = a10;
	m[1][1] = a11;
}


txMatrix2::~txMatrix2(void)
{
}


txVec2 txMatrix2::Solve(const txVec2 &v) {
	txVec2 rtnv;
	double det  = m[0][0]*m[1][1] - m[0][1]*m[1][0];
	double detX = v.X()*m[1][1] - m[0][1]*v.Y();
	double detY = m[0][0]*v.Y() - v.X()*m[1][0];
	double x = detX/det;
	double y = detY/det;
	assert(abs(det)>MATRIX_DETERMINE_PRECISION);
	rtnv.SetX(x);
	rtnv.SetY(y);
	return rtnv;
}