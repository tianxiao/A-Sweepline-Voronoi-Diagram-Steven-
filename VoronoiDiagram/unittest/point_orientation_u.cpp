#include "stdafx.h"
#include <math.h>
#include "../thirdparty/gtest-1.6.0/include/gtest.h"
#include "../VoronoiDiagramDLL/VoronoiBuilder.h"

TEST(txVoronoiBuilder, PointOrientationChecking ) {
	txVertex v0 = {0, 0}, v1 = {2, 0}, v2 = {2, 2};
	txVertex v10 = {0, 0}, v11 = {2, 0}, v12 = {2, -2};
	txVertex v20 = {0, 0}, v21 = {2, 0}, v22 = {4, 0};
	EXPECT_TRUE(P_ORIENTATION_LEFT == txVoronoiBuilder::PointOrientationChecking(v0,v1,v2));
	EXPECT_TRUE(P_ORIENTATION_RIGHT == txVoronoiBuilder::PointOrientationChecking(v10,v11,v12));
	EXPECT_TRUE(P_COLLINEAR == txVoronoiBuilder::PointOrientationChecking(v20,v21,v22));

}