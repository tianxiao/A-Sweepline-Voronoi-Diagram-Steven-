#pragma once
#include <vector>
#include <queue>
#include <stdlib.h>
#include <list>
#include <map>
#include "halfedgeprimitive.h"

#define PRECISION_INFINIT -1e20

class txMesh;
struct txPriorityNode;

typedef enum txVoronoiEventType{
	SITE_EVENT,
	CIRCLE_EVENT,
} txVoronoiEventType;

typedef struct txArc{
	int             id;
	txVertex        *pV;
	//txPriorityNode  *circleEvent;
	int             PQId;
	txArc(txVertex *pV_):pV(pV_),PQId(-1){};
} txArc;

typedef struct txPriorityNode{
	int                  id;
	txVertex             *pV;  // point to the site
	txVoronoiEventType   eventType;
	//txArc                *al;
	//txArc                *am;
	//txArc                *ar;
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


class txVoronoiBuilder
{

public:
	txVoronoiBuilder(int numOfSites);
	~txVoronoiBuilder(void);

	void AddSites(const txVertex &vertex){ sitesList.push_back(vertex); };

	void Build();

private:
	void InitialEventQueue();
	void HandleSiteEvent(const txPriorityNode &siteEvent);
	void HandleCircleEvent(const txPriorityNode &cirlceEvent);
	void DeleteFalseAlarmCircleEvent(int circleId);
	void InsertEvent(const txPriorityNode &pevent);
	static void Bisector(const txVertex &v0, const txVertex &v1, txEdge &edge);
	static void Circle(const txVertex &n0, const txVertex &n1, const txVertex &n2, double &y);
	bool GetTripleAsLeft(BLIt middle, BLIt &l, BLIt &ll);
	bool GetTripleAsMiddle(BLIt middle, BLIt &l, BLIt &r);
	bool GetTripleAsRight(BLIt middle, BLIt &r, BLIt &rr);
	void InserteArc(const txArc &arc);
	BLIt GetArcIterator(const txArc &arc);


private:
	txMesh                               *mesh;
	std::vector<txVertex>                sitesList;
	std::list<txPriorityNode>            eventQueue;
	std::list<txEdge>                    edgeList;
	std::list<txArc>                     beachLine;
	int                                  arcCount;
	int                                  eventCount;
};

