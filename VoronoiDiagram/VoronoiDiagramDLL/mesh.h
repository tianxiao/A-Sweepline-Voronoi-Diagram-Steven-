#pragma once
#ifndef __MESH_HEADERFILE__
#define __MESH_HEADERFILE__
#include <list>
#include <vector>
#include "halfedgeprimitive.h"

class txMesh{

public:
	txMesh (int numOfSites);

	void AddHalfEdge(const txHalfEdge &halfEdge);
	void AddVertex(const txVertex &vertex);
private:
	std::list<txVertex>        vertexList;
	std::list<txHalfEdge>      halfEdgeList;
	std::vector<txFace>        faceList;


};



#endif // __MESH_HEADERFILE__