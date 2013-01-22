#pragma once
#include "import.h"


class R_DECLDIR txVec2
{
public:
	txVec2(void);
	txVec2(double a0, double a1);
	~txVec2(void);
	double X() const { return v[0]; };
	double Y() const { return v[1]; };
	void SetX(double x) { v[0] = x; };
	void SetY(double y) { v[1] = y; };

private:
	double v[2];
};

