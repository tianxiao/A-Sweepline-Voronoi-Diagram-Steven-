#include "StdAfx.h"
#include "mesh.h"

txMesh::txMesh (int numOfSites){
	faceList.reserve(numOfSites);
	for (int i=0; i<numOfSites; i++ ){
		txFace currentFace;
		currentFace.id = i;
		currentFace.pArbitrayHalfEdge = NULL;
		faceList.push_back(currentFace);
	}
}

void txMesh::AddHalfEdge(const txHalfEdge &halfEdge){
	halfEdgeList.push_back(halfEdge);
}

void txMesh::AddVertex(const txVertex &vertex){
	vertexList.push_back(vertex);
}