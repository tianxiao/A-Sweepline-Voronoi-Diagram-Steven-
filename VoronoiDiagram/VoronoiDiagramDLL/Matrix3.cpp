#include "StdAfx.h"
#include "Matrix3.h"


txMatrix3::txMatrix3(void)
{
	m[0][0] = 0.0; m[0][1] = 0.0; m[0][2] = 0.0;
	m[1][0] = 0.0; m[1][1] = 0.0; m[1][2] = 0.0;
	m[2][0] = 0.0; m[2][1] = 0.0; m[2][2] = 0.0;
}


txMatrix3::txMatrix3(
	double m00, double m01, double m02,
	double m10, double m11, double m12,
	double m20, double m21, double m22) {

	m[0][0] = m00; m[0][1] = m01; m[0][2] = m02;
	m[1][0] = m10; m[1][1] = m11; m[1][2] = m12;
	m[2][0] = m20; m[2][1] = m21; m[2][2] = m22;
}

txMatrix3::~txMatrix3(void)
{
}


double txMatrix3::Determinant( void ) const {
	double det200 = m[1][1]*m[2][2] - m[1][2]*m[2][1];
	double det201 = m[1][0]*m[2][2] - m[1][2]*m[2][0];
	double det202 = m[1][0]*m[2][1] - m[1][1]*m[2][0];

	double determin = m[0][0]*det200 - m[0][1]*det201 + m[0][2]*det202;
	return determin;
}