#pragma once
#include <vector>
#include <queue>
#include <stdlib.h>
#include <list>
#include <map>
#include "import.h"
#include "halfedgeprimitive.h"

#define PRECISION_INFINIT -1e20

#pragma warning(push)
#pragma warning(disable:4251)

class txMesh;
struct txPriorityNode;
struct txEdge;

typedef enum txVoronoiEventType{
	SITE_EVENT,
	CIRCLE_EVENT,
} txVoronoiEventType;

typedef struct txBreakPoint{
	int        id;
	txVertex   v;
} txBreakPoint;

// The prabola intersection will need to
// consider the parameter ranges
// Here I only consider the reference line is parallel 
// to the Y Axis, and the range is from -Infi, Infi
typedef enum txParabolaIntersectionType{
	PARABOLA_INTECT_ONE,
	PARABOLA_INTECT_TWO,
	PARABOLA_INTECT_NONE
} txParabolaIntersectionType;

typedef struct txArc{
	int             id;
	// The interval right value is the pre node left value
	// If the pre node is NULL then it's set to PRECISION_INFINIT this value 
	// will be changed to the bounding box
	// The last one will be set to +PRECISION_INFINIT which will be changed to 
	// the bounding box value too.
	// meaning leftmost:[-Infinite,l0]  |   1[l0,l1]   |    2[l1,l2] ...  [ln,+Infinite]
	// when call the breakpoints update function the leftmost will never be updated!
	// cause it is always -Infinite
	double          leftValue;   
	txVertex        *pV;
	//txPriorityNode  *circleEvent;
	int             PQId;
	//int             leftBPId;
	//int             rightBPId;
	int             edgeId;
	txArc(txVertex *pV_):pV(pV_),PQId(-1),edgeId(-1){};
} txArc;

typedef struct txEdge{
	int        id;
	txVertex   *pLSite;
	txVertex   *pRSite;
	int        startBPId;    // start txBreakPoint id
	int        endBPId;    // end txBreakPoint id
	double a,b,c;
	txEdge() :pLSite(NULL),pRSite(NULL), startBPId(-1), endBPId(-1), a(0.0), b(0.0), c(0.0){};
} txEdge;

typedef struct txPriorityNode{
	int                  id;
	txVertex             *pV;  // point to the site
	txVoronoiEventType   eventType;
	int                  aLId;
	int                  aMId; // The middle arc will disappear when circle event happened
	int                  aRId;
	double               circleBottomY;
	txPriorityNode(txVertex *pV_, txVoronoiEventType eventType_)
		:pV(pV_)
		,eventType(eventType_)
		,circleBottomY(PRECISION_INFINIT)
	{}

	void PrintNode(){
		printf("%d--%d\n",pV->x,pV->y);
	}
} txPriorityNode;

struct txPriorityNodeCmp{
	bool operator()( txPriorityNode *l,  txPriorityNode *r) const {
		if (l->pV->y > r->pV->y){
			return true;
		} else {
			return false;
		}
	}
};

//bool priorityNodeCmp(txPriorityNode l, txPriorityNode r){
//	return txPriorityNodeCmp()(l,r);
//}

typedef std::list<txArc> BLList;
typedef BLList::iterator BLIt;
typedef std::list<txPriorityNode> PQList;
typedef PQList::iterator PQIt;
typedef std::list<txBreakPoint> BPList;


class R_DECLDIR txVoronoiBuilder
{

public:
	txVoronoiBuilder(int numOfSites);
	~txVoronoiBuilder(void);

	void AddSites(const txVertex &vertex){ sitesList.push_back(vertex); };

	void Build();

public:
	static void Bisector(const txVertex &v0, const txVertex &v1, txEdge &edge);
	static void Circle(const txVertex &n0, const txVertex &n1, const txVertex &n2, double &y);
	static void CalculateTwoParabolaIntersectionPoints(const txVertex &p0, const txVertex &p1, double ly0, double ly1, txVertex &v0, txVertex &v1, txParabolaIntersectionType &type);


private:
	void InitialEventQueue();
	void HandleSiteEvent(const txPriorityNode &siteEvent);
	void HandleCircleEvent(const txPriorityNode &cirlceEvent);
	void DeleteFalseAlarmCircleEvent(int circleId);
	void InsertEvent(const txPriorityNode &pevent);
	bool GetTripleAsLeft(BLIt middle, BLIt &l, BLIt &ll);
	bool GetTripleAsMiddle(BLIt middle, BLIt &l, BLIt &r);
	bool GetTripleAsRight(BLIt middle, BLIt &r, BLIt &rr);
	void InserteArc(const txArc &arc);
	BLIt GetArcIterator(const txArc &arc);
	void UpdateBreakPointsList(double bottomY);
	void InsertEdge(const txEdge &edge);
	int GetUpperArcId(double x);
	int InsertNewArc(const txPriorityNode &siteEvent);
	BLIt GetArcFromId(int id);
	void CheckCircleEvent(int newArcId);
	void AddCircleEvent(BLIt lIt, BLIt mIt, BLIt rIt);


private:
	txMesh                               *mesh;
	std::vector<txVertex>                sitesList;
	std::list<txPriorityNode>            eventQueue;
	std::list<txEdge>                    edgeList;
	std::list<txArc>                     beachLine;
	int                                  arcCount;
	int                                  eventCount;
	int                                  edgeCount;
	BPList                               bpList;
};

#pragma warning(pop)

