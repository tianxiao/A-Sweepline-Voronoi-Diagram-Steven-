#include "StdAfx.h"
#include "Vec2.h"


txVec2::txVec2(void)
{
	v[0] = 0.0;
	v[1] = 0.0;
}

txVec2::txVec2(double a0, double a1) {
	v[0] = a0;
	v[1] = a1;
}

txVec2::~txVec2(void)
{
}
