// VoronoiDiagram.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "VoronoiBuilder.h"


int _tmain(int argc, _TCHAR* argv[])
{
	const int numOfSites = 3;
	txVoronoiBuilder builder(numOfSites);
	txVertex vList[numOfSites] = { {0,0},{0,4},{1,1} };
	for (int i=0; i<numOfSites; i++) {
		builder.AddSites(vList[i]);
	}

	builder.Build();

	return 0;
}

