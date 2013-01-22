#include "stdafx.h"
#include <math.h>
#include "../thirdparty/gtest-1.6.0/include/gtest.h"
#include "../VoronoiDiagramDLL/VoronoiBuilder.h"

#define LINE_PARAMETER_PRECISION 1e-17
#define CIRCLE_CENTERPOINT_PRECISION 1e-17

TEST(txVoronoiBuilder, Bisector) {
	txVertex v0 = {0,4};
	txVertex v1 = {2,1};
	txEdge e;
	txVoronoiBuilder::Bisector(v0, v1, e);

	EXPECT_TRUE(abs(e.a+2.0)<LINE_PARAMETER_PRECISION);
	EXPECT_TRUE(abs(e.b-3.0)<LINE_PARAMETER_PRECISION);
	EXPECT_TRUE(abs(e.c+5.5)<LINE_PARAMETER_PRECISION);

}

TEST(txVoronoiBuilder, Circle) {
	txVertex n0 = {0,4};
	txVertex n1 = {2,1};
	txVertex n2 = {1,0};
	double bottomY;

	txVoronoiBuilder::Circle(n0, n1, n2, bottomY);
	EXPECT_TRUE(abs(bottomY-(1.9-2.1023796041628637))<CIRCLE_CENTERPOINT_PRECISION);
}