#include "stdafx.h"
#include <math.h>
#include "../thirdparty/gtest-1.6.0/include/gtest.h"
#include "../VoronoiDiagramDLL/Matrix2.h"
#include "../VoronoiDiagramDLL/Vec2.h"

#define MATRIX_PRECISION 1e-17

TEST(txMatrix2, Solve){
	// | 1, 2 | x {11} 
	// | 4, 5 | y {32} 
	// x=3.0  y=4.0
	txVec2 v(11,32);
	txMatrix2 m(1,2,4,5);

	txVec2 resultv = m.Solve(v);

	EXPECT_TRUE(abs(resultv.X()-3.0)<MATRIX_PRECISION);
	EXPECT_TRUE(abs(resultv.Y()-4.0)<MATRIX_PRECISION);
}