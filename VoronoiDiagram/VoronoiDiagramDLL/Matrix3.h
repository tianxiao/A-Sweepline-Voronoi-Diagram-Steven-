#pragma once
#include "import.h"

class R_DECLDIR txMatrix3
{
public:
	txMatrix3(void);
	txMatrix3(
		double m00, double m01, double m02,
		double m10, double m11, double m12,
		double m20, double m21, double m22);


	~txMatrix3(void);

	double Determinant( void ) const ;


private:
	double m[3][3];
};

