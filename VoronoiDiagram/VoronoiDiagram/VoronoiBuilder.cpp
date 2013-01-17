#include "StdAfx.h"
#include "VoronoiBuilder.h"
#include <algorithm>
#include <assert.h>
#include <math.h>
#include "mesh.h"

#define PRESISION_OF_BISECTOR 1e-7
#define PRESISION_OF_LINEPARALELL_DETERMIN 1e-7


txVoronoiBuilder::txVoronoiBuilder(int numOfSites)
{
	mesh = new txMesh(numOfSites);
	arcCount = 0;
	eventCount = 0;
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
	int siteX = siteEvent.pV->x;
	txArc newArc(siteEvent.pV);
	newArc.id = arcCount++;
	// InsertArc will rtn a newArc's iterator
	InserteArc(newArc);
	BLIt newArcIt = GetArcIterator(newArc);
	BLIt upperBIt = newArcIt;
	upperBIt++; // upper upper x >= self x
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
void txVoronoiBuilder::InserteArc(const txArc &arc){
	BLIt rtn;
	for ( rtn=beachLine.begin(); rtn!=beachLine.end(); rtn++ ) {
		if ( arc.pV->x < rtn->pV->x ) {
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