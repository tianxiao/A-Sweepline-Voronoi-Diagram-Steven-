#pragma once
#include "import.h"
#include "Vec2.h"

class R_DECLDIR txMatrix2
{
public:
	txMatrix2(void);
	txMatrix2(double a00, double a01, double a10, double a11);
	~txMatrix2(void);

	txVec2 Solve(const txVec2 &v);

private:
	double m[2][2];
};


