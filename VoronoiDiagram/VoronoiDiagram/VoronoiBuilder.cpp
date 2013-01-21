#include "StdAfx.h"
#include "VoronoiBuilder.h"
#include <algorithm>
#include <assert.h>
#include <math.h>
#include "mesh.h"

#define PRESISION_OF_BISECTOR 1e-7
#define PRESISION_OF_LINEPARALELL_DETERMIN 1e-7
#define PRESISION_OF_PARABOLA_INTERSECTION 1e-7
#define PRESISION_OF_PARABOLA_INTERSECTION_DELTA 1e-7


txVoronoiBuilder::txVoronoiBuilder(int numOfSites)
{
	mesh = new txMesh(numOfSites);
	arcCount = 0;
	eventCount = 0;
	edgeCount = 0;
}


txVoronoiBuilder::~txVoronoiBuilder(void)
{
	delete mesh;
	mesh = NULL;
}

void txVoronoiBuilder::Build(){
	InitialEventQueue();
	while (!eventQueue.empty()){
		txPriorityNode currentEvent = *(eventQueue.begin());
		eventQueue.pop_front();
		if (SITE_EVENT==currentEvent.eventType)
		{
			HandleSiteEvent(currentEvent);
		} else {
			HandleCircleEvent(currentEvent);
		}

	}
}


void txVoronoiBuilder::InitialEventQueue(){
	for (size_t i=0; i<sitesList.size(); i++){
		txPriorityNode newEvent(&sitesList[i],SITE_EVENT);
		newEvent.id = eventCount++;
		InsertEvent(newEvent);
	}
}

void txVoronoiBuilder::HandleSiteEvent(const txPriorityNode &siteEvent){
	double siteX = siteEvent.pV->x;
	
	UpdateBreakPointsList(siteX);
	txArc newArc(siteEvent.pV);
	newArc.leftValue = siteEvent.pV->x;
	newArc.id = arcCount++;
	// InsertArc will rtn a newArc's iterator
	if ( beachLine.size() == 0 ) { beachLine.push_back(newArc); return; }
	InserteArc(newArc);
	BLIt newArcIt = GetArcIterator(newArc);
	BLIt upperBIt = newArcIt;
	upperBIt++; // upper upper x >= self x

	// Insert new edge
	txEdge edge;
	edge.pLSite = siteEvent.pV;
	if ( upperBIt !=  beachLine.end() ) {
		edge.pRSite = upperBIt->pV;
	}
	edge.id = edgeCount++;

	// Assign the edge id to the arc 
	newArcIt->edgeId = edgeCount;
	if ( beachLine.size() ==1 ) return;
	// check corresponding circle event see if it is false alarm
	if ( upperBIt != beachLine.end() ) {
		if ( !upperBIt->PQId ) {
			DeleteFalseAlarmCircleEvent(upperBIt->PQId);
		}
	}

	BLIt l, ll;
	if ( GetTripleAsLeft(newArcIt, l, ll) ) {
		// check cirlce event
		printf("l-ll\n");
	}
	BLIt r, rr;
	if ( GetTripleAsRight(newArcIt, r, rr) ) {
		// check circle event
		printf("r-rr\n");
	}
	BLIt ml, mr;
	if ( GetTripleAsMiddle(newArcIt, ml, mr) ) {
		// check circle event
		printf("middle\n");
		printf("%d-%d-%d\n",ml->pV->x, newArcIt->pV->x, mr->pV->x);
		double bottomY;
		Circle(*newArcIt->pV, *ml->pV, *mr->pV,bottomY);
		txPriorityNode circleEvent(NULL,CIRCLE_EVENT);
		circleEvent.id = eventCount++;
		circleEvent.circleBottomY = bottomY;
		InsertEvent(circleEvent);
	}

}

void txVoronoiBuilder::HandleCircleEvent(const txPriorityNode &cirlceEvent){

}

// When the arc related circel event be delete
// Anything else we must do ?
// This question leave to the handl circle event.
void txVoronoiBuilder::DeleteFalseAlarmCircleEvent(int circleId){
	PQIt eraseIt = eventQueue.begin();
	while (eraseIt!=eventQueue.end()) {
		if (eraseIt->id == circleId) break;
		eraseIt++;
	}
	eventQueue.erase(eraseIt);
}

void txVoronoiBuilder::InsertEvent(const txPriorityNode &pevent){
	if (!eventQueue.size()) {
		eventQueue.push_back(pevent);
		return;
	}
	PQIt insert = eventQueue.begin();
	while (insert!=eventQueue.end()) {
		if (pevent.pV->y>insert->pV->y) break;
		insert++;
	}
	std::list<txPriorityNode> insertlist;
	insertlist.push_back(pevent);
	eventQueue.insert(insert,insertlist.begin(),insertlist.end());
}

void txVoronoiBuilder::Bisector(const txVertex &v0, const txVertex &v1, txEdge &edge){
	double detv01 = (v0.x-v1.x)*(v0.x-v1.x)+(v0.y-v1.y)*(v0.y-v1.y);
	assert(abs(detv01)>PRESISION_OF_BISECTOR);  // check if identical at precision 
	edge.a = v0.y-v1.y;
	edge.b = -(v0.x-v1.x);
	edge.c = -(edge.a*(v0.x+v1.x)*0.5 + edge.b*(v0.y+v1.y)*0.5);
}

void txVoronoiBuilder::Circle(const txVertex &n0, const txVertex &n1, const txVertex &n2, double &y){
	txEdge e0;
	txEdge e1;
	Bisector(n0,n1,e0);
	Bisector(n1,n2,e1);
	// PRESISION_OF_LINEPARALELL_DETERMIN
	double det = e0.a*e1.b-e0.b*e1.a;
	assert(abs(det)>PRESISION_OF_LINEPARALELL_DETERMIN); // Need return a false 3 points planar
	double centerX = -e0.c/det;
	double centerY = -e1.c/det;
	double radius = sqrt((n0.x-centerX)*(n0.x-centerX)+(n1.y-centerY)*(n1.y-centerY));

	y = centerY - radius;
}


// Insert the arc as the x coordinate increase!
// The arc in list is ordered in increase order.
// So the arc.leftValue "<" ...
// Insert a new arc will lead one old arc split to two arc
// FIXME!!!
void txVoronoiBuilder::InserteArc(const txArc &arc){
	BLIt rtn;
	for ( rtn=beachLine.begin(); rtn!=beachLine.end(); rtn++ ) {
		if ( arc.leftValue < rtn->leftValue ) { 
			break;
		}
	} 
	BLList tempList;
	tempList.push_back(arc);

	beachLine.insert(rtn, tempList.begin(), tempList.end());
	// check if rtn validate after the list have been updated
	// OK rtn is invalidate!
}

BLIt txVoronoiBuilder::GetArcIterator(const txArc &arc){
	for ( BLIt it = beachLine.begin(); it!=beachLine.end(); it++ ) {
		if ( arc.id == it->id ) return it;
	}
	return beachLine.end();
}

bool txVoronoiBuilder::GetTripleAsLeft(BLIt middle, BLIt &l, BLIt &ll){
	l=middle;
	if ( middle==beachLine.begin() ) return false;
	middle--;
	if ( middle==beachLine.begin() ) return false;
	middle--;
	if ( middle==beachLine.begin() ) return false;
	l--;
	ll = l;
	ll--;
	return true;
}

bool txVoronoiBuilder::GetTripleAsMiddle(BLIt middle, BLIt &l, BLIt &r){
	l=r=middle;
	if ( middle==beachLine.begin() )  return false;
	middle++;
	if ( middle==beachLine.end() ) return false;
	l--;
	r++;
	return true;
}

bool txVoronoiBuilder::GetTripleAsRight(BLIt middle, BLIt &r, BLIt &rr){
	r = middle;
	if ( middle==beachLine.end() ) return false;
	middle++;
	if ( middle==beachLine.end() ) return false;
	middle++;
	if ( middle==beachLine.end() )   return false;
	r++;
	rr=r;
	rr++;
	return true;
}

// call the following functin when comes to new site event.
// new site event wont change the arc topology
// Only the Circle event will.
// guess what the arc will not be 2! so check 2 is unecessary!
void txVoronoiBuilder::UpdateBreakPointsList(double bottomY){
	BLIt blit = beachLine.begin();
	if ( blit == beachLine.end() ) return;
	blit++; // first arc need not to be updated see the txArc structure definatio
	while (blit!=beachLine.end()) {
		BLIt pre, next; pre=next=blit;
		pre--,next++;
		txVertex v0;
		txVertex v1;
		txParabolaIntersectionType type;
		CalculateTwoParabolaIntersectionPoints(*pre->pV,*blit->pV,bottomY, bottomY, v0, v1, type);
		assert(type==PARABOLA_INTECT_TWO);
		// check if the pre and nex arc are belong to the same site (parabola)
		if ( pre->pV==next->pV ) {
			blit->leftValue = v0.x;
			next->leftValue = v1.x;
			blit++;blit++; // skip the next arc cause we have already calculated the next one in this step
		} else {
			// let the left arc intersect with the middle pick the maximum value
			blit->leftValue = v1.x;
			blit++;
		}
	}
}

// keep the v0.x < v1.x
void txVoronoiBuilder::CalculateTwoParabolaIntersectionPoints(const txVertex &p0, const txVertex &p1, double ly0, double ly1, txVertex &v0, txVertex &v1, txParabolaIntersectionType &type){
	double x1, x2, y1, y2;
	double a1, a2;
	double a, b, c;
	x1 = p0.x; y1 = p0.y;
	x2 = p1.x; y2 = p1.y;
	assert(abs(ly0-y1)>PRESISION_OF_PARABOLA_INTERSECTION);
	assert(abs(ly1-y2)>PRESISION_OF_PARABOLA_INTERSECTION);
	a1 = 0.5/(y1-ly0);
	a2 = 0.5/(y2-ly1);
	a = a1 + a2;
	b = -2*(a1*x1 + a2*x2);
	c = (a1*x1*x1 + a2*x2*x2 + y1 + y2);

	double delta = b*b - 4*a*c;
	assert (delta>0);

	if (delta<PRESISION_OF_PARABOLA_INTERSECTION_DELTA) {
		v0.x = -0.5*b/a;
		v0.y = a1*(v0.x - x1)*(v0.x - x1) + y1;
		v1 = v0;
		type = PARABOLA_INTECT_ONE;
	} else {
		double sqrtdelta = sqrt(delta);
		v0.x = 0.5*(-b+sqrtdelta)/a;
		v0.y = a1*(v0.x-x1)*(v0.x-x1) + y1;
		v1.x = 0.5*(b+sqrtdelta)/1;
		v1.y = a2*(v1.x-x2)*(v1.x-x2) + y2;
		type = PARABOLA_INTECT_TWO;
		if ( v0.x>v1.x ) {
			std::swap(v0, v1);
		}
	}
	
}

void txVoronoiBuilder::InsertEdge(const txEdge &edge){
	edgeList.push_back(edge);
}

int txVoronoiBuilder::GetUpperArcId(double x){
	for (BLIt it=beachLine.begin(); it!=beachLine.end(); it++) {
		if ( x < it->leftValue ) {
			return it->id;
		}
	}
	return -1;
}

// doesn't like the InserteArc function 
// It will 
// 1)insert the degenerate parabola ( vertical line )
// 2)delete the upper arc and 
// 3)create two new arc based on the delete arc
void txVoronoiBuilder::InsertNewArc(const txPriorityNode &siteEvent) {
	double siteX = siteEvent.pV->x;
	int upperArcId = GetUpperArcId(siteX);
	BLIt upperArcIt = GetArcFromId(upperArcId);
	assert(upperArcIt!=beachLine.end());
	txArc newArc(siteEvent.pV);
	newArc.leftValue = siteX;
	newArc.id = arcCount++;

	txArc leftArc(upperArcIt->pV);
	leftArc.leftValue = upperArcIt->leftValue;
	newArc.id = arcCount++;

	txArc rightArc(upperArcIt->pV);
	rightArc.leftValue = siteX;
	rightArc.id = arcCount++;

	BLList tempList;
	tempList.push_back(leftArc);
	tempList.push_back(newArc);
	tempList.push_back(rightArc);
	// Insert the newly created 3 arcs
	beachLine.insert(upperArcIt, tempList.begin(), tempList.end());
	// Delete the old arc ( the upper arc )
	// Befor erase the upper arc we should check to see if it contains the circle event
	// And this may contain one circle events, It may contain more than one circle event
	// If it contain the circle event then this event is false alarm circle event
	DeleteFalseAlarmCircleEvent(upperArcIt->PQId);

	beachLine.erase(upperArcIt);

}

BLIt txVoronoiBuilder::GetArcFromId(int id) {
	for (BLIt it=beachLine.begin(); it!=beachLine.end(); it++ ) {
		if ( it->id == id ) return it;
	}

	return beachLine.end();
}
