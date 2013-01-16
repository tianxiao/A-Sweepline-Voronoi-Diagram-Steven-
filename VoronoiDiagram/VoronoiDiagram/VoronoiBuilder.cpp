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
			
		}

	}
}


void txVoronoiBuilder::InitialEventQueue(){
	for (size_t i=0; i<sitesList.size(); i++){
		InsertEvent(txPriorityNode(&sitesList[i],SITE_EVENT));
	}
}

void txVoronoiBuilder::HandleSiteEvent(const txPriorityNode &siteEvent){
	int siteX = siteEvent.pV->x;
	beachLine.insert(std::pair<int, txArc>(siteX,txArc(siteEvent.pV,NULL) ));
	if ( beachLine.size() ==1 ) return;
	// check corresponding circle event see if it is false alarm
	txArc *currentAboveArc = NULL;
	BeachIt upperBIt = beachLine.upper_bound(siteX);
	if ( upperBIt != beachLine.end() ) {
		currentAboveArc = &(upperBIt->second);
		if ( currentAboveArc->circleEvent ) {
			DeleteFalseAlarmCircleEvent(currentAboveArc->circleEvent);
		}
	}

	// Handle ll
	txArc *lArc = NULL;
	txArc *llArc = NULL;
	if ( beachLine.lower_bound(siteX) != beachLine.end() ) {
		lArc = &(beachLine.lower_bound(siteX)->second);
		if ( beachLine.lower_bound(lArc->pV->x) != beachLine.end() ){
			llArc = &(beachLine.lower_bound(lArc->pV->x)->second);
		}
	}
	if (lArc!=NULL && llArc!=NULL && lArc!=llArc) {
		
	}

	// Handle rr
	txArc *rArc = NULL;
	txArc *rrArc = NULL;
	if ( beachLine.upper_bound(siteX) != beachLine.end() ) {
		rArc = &(beachLine.upper_bound(siteX)->second);
		if ( beachLine.upper_bound(rArc->pV->x) != beachLine.end() ){
			rrArc = &(beachLine.upper_bound(rArc->pV->x)->second);
		}
	}
	if (rArc!=NULL && rrArc!=NULL && rArc!=rrArc) {
	
	}

	// Handle middle
	txArc *lmArc = NULL;
	txArc *rmArc = NULL;
	if ( beachLine.lower_bound(siteX) != beachLine.end() ) {
		lmArc = &(beachLine.lower_bound(siteX)->second);
		if (beachLine.upper_bound(siteX) != beachLine.end() ) {
			rmArc = &(beachLine.upper_bound(siteX)->second);
		}
	}
	if (lmArc!=NULL && rmArc!=NULL && lmArc!=rmArc) {
		printf("Check Circle...\n");
		double y;
		Circle(*lmArc->pV,*siteEvent.pV,*rmArc->pV,y);
	}


}

void txVoronoiBuilder::HandleCircleEvent(const txPriorityNode &cirlceEvent){

}

void txVoronoiBuilder::DeleteFalseAlarmCircleEvent(txPriorityNode *circleEvent){
	PQIt eraseIt = eventQueue.begin();
	while (eraseIt!=eventQueue.end()) {
		if (&(*eraseIt)==circleEvent) break;
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
	assert(detv01<PRESISION_OF_BISECTOR);  // check if identical at precision 
	edge.a = v0.y-v1.y;
	edge.b = v0.x-v0.x;
	edge.c = -(edge.a*(v0.x+v1.x)*0.5 + edge.b*(v0.y+v1.y)*0.5);
}

void txVoronoiBuilder::Circle(const txVertex &n0, const txVertex &n1, const txVertex &n2, double &y){
	txEdge e0;
	txEdge e1;
	Bisector(n0,n1,e0);
	Bisector(n1,n2,e1);
	// PRESISION_OF_LINEPARALELL_DETERMIN
	double det = e0.a*e1.b-e0.b*e1.a;
	assert(det<PRESISION_OF_LINEPARALELL_DETERMIN);
	double centerX = -e0.c/det;
	double centerY = -e1.c/det;
	double radius = sqrt((n0.x-centerX)*(n0.x-centerX)+(n1.y-centerY)*(n1.y-centerY));

	y = centerY - radius;
}

