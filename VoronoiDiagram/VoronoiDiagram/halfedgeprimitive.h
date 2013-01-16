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
	int x, y;
} txVertex;

typedef struct txHalfEdge{
	txVertex         *pVertex;          // The vertex it point to
	txFace           *pFace;            // Fact it belong to  
	txHalfEdge       *pNextHalf;        // The next halfedge CCW
	txHalfEdge       *pTwinHalf;        // The 'twin' halfedge
	txHalfEdge       *PPre;	            // The pre Half [Optional]
} txHalfEdge;

typedef struct txEdge{
	txVertex   *pLSite;
	txVertex   *pRSite;
	txVertex   *pS;
	txVertex   *pE;    // This edge is the bisector of pLSite & pRSite
} txEdge;

#endif // __HALFEDGEPRIMITIVE_HEADERFILE__