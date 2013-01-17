#pragma once
#include <vector>
#include <queue>
#include <stdlib.h>
#include <list>
#include <map>
#include "halfedgeprimitive.h"

class txMesh;
struct txPriorityNode;



typedef enum txVoronoiEventType{
	SITE_EVENT,
	CIRCLE_EVENT,
	FALSE_ALARM
} txVoronoiEventType;

typedef struct txArc{
	int             id;
	txVertex        *pV;
	txPriorityNode  *circleEvent;
	txArc(txVertex *pV_, txPriorityNode *circleEvent_):pV(pV_),circleEvent(circleEvent_){};
} txArc;

typedef struct txPriorityNode{
	txVertex             *pV;  // point to the site
	txVoronoiEventType   eventType;
	txArc                *al;
	txArc                *am;
	txArc                *ar;
	txPriorityNode(txVertex *pV_, txVoronoiEventType eventType_)
		:pV(pV_)
		,eventType(eventType_){}

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
	void DeleteFalseAlarmCircleEvent(txPriorityNode *circleEvent);
	void InsertEvent(const txPriorityNode &pevent);
	static void Bisector(const txVertex &v0, const txVertex &v1, txEdge &edge);
	static void Circle(const txVertex &n0, const txVertex &n1, const txVertex &n2, double &y);
	//bool GetTripleAsLeft();
	void InserteArc(const txArc &arc);
	BLIt GetArcIterator(const txArc &arc);


private:
	txMesh                               *mesh;
	std::vector<txVertex>                sitesList;
	std::list<txPriorityNode>            eventQueue;
	std::list<txEdge>                    edgeList;
	std::list<txArc>                     beachLine;
	int                                  arcCount;

};

