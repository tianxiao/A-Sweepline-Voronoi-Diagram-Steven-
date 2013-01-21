#ifndef __HALFEDGEPRIMITIVE_HEADERFILE__
#define __HALFEDGEPRIMITIVE_HEADERFILE__

struct txFace;
struct txHalfEdge;
struct txVertex;

typedef struct txFace{
	int id;                    // corresponding the Voronoi Diagram Sites
	txHalfEdge       *pArbitrayHalfEdge;
} txFace;

typedef struct txVertex{
	double x, y;
} txVertex;

typedef struct txHalfEdge{
	txVertex         *pVertex;          // The vertex it point to
	txFace           *pFace;            // Fact it belong to  
	txHalfEdge       *pNextHalf;        // The next halfedge CCW
	txHalfEdge       *pTwinHalf;        // The 'twin' halfedge
	txHalfEdge       *PPre;	            // The pre Half [Optional]
} txHalfEdge;



#endif // __HALFEDGEPRIMITIVE_HEADERFILE__