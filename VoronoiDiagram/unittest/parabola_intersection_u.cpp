#include "stdafx.h"
#include "../thirdparty/gtest-1.6.0/include/gtest.h"
#include "../VoronoiDiagramDLL/VoronoiBuilder.h"

TEST(VoronoiBuilder, CalculateTwoParabolaIntersectionPoints) {
	txVertex p0 = {0,4};
	txVertex p1 = {2,1};
	double ly0 = 0.0;
	double ly1 = 0.0;
	txVertex v0;
	txVertex v1;
	txParabolaIntersectionType type;
	txVoronoiBuilder::CalculateTwoParabolaIntersectionPoints(p0,p1,ly0,ly1,v0,v1,type);

	// add test case 
	// check v0 ,v1 and the type
}