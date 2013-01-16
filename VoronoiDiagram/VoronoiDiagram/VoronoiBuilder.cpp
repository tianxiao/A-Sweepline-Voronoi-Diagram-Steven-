#include "StdAfx.h"
#include "VoronoiBuilder.h"
#include <algorithm>
#include "mesh.h"


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
		txPriorityNode *currentEvent = *(eventQueue.begin());
		eventQueue.pop_front();
		if (SITE_EVENT==currentEvent->eventType)
		{
			HandleSiteEvent(*currentEvent);
		} else {
			
		}

	}
}


void txVoronoiBuilder::InitialEventQueue(){
	for (size_t i=0; i<sitesList.size(); i++){
		//eventQueue.push(txPriorityNode(&sitesList[i],SITE_EVENT));
		eventPool.push_back(txPriorityNode(&sitesList[i],SITE_EVENT));
	}

	UpdatePriorityQueue();
	// debug Priority queue
	//while (!eventQueue.empty()){
	//	eventQueue.top().PrintNode();
	//	eventQueue.pop();
	//}
}

void txVoronoiBuilder::HandleSiteEvent(const txPriorityNode &siteEvent){
	beachLine.insert(std::pair<int, txArc>(siteEvent.pV->x,txArc(siteEvent.pV,NULL) ));
	// check corresponding circle event see if it is false alarm
	txArc *currentAboveArc = NULL;
	BeachIt upperBIt = beachLine.upper_bound(siteEvent.pV->x);
	if ( upperBIt != beachLine.end() ) {
		currentAboveArc = &(upperBIt->second);
		if ( currentAboveArc!=NULL) {
			
		}
	}

	txArc *lArc = NULL;
	txArc *llArc = NULL;
	if ( beachLine.lower_bound(siteEvent.pV->x) != beachLine.end() ) {
		lArc = &(beachLine.lower_bound(siteEvent.pV->x)->second);
		if ( beachLine.lower_bound(lArc->pV->x) != beachLine.end() ){
			llArc = &(beachLine.lower_bound(lArc->pV->x)->second);
		}
	}
	if (lArc!=NULL&&llArc!=NULL) {
		
	}
}

void txVoronoiBuilder::HandleCircleEvent(const txPriorityNode &cirlceEvent){

}

void txVoronoiBuilder::DeleteFalseAlarmCircleEvent(txPriorityNode *circleEvent){
	//for (size_t i=0; i<eventQueue.size(); i++) {
	//	if ( eventQueue[i]
	//}
	//eventQueue.
}

void txVoronoiBuilder::UpdatePriorityQueue(){
	std::vector<txPriorityNode*> nodelist;
	nodelist.reserve(eventPool.size());
	for (std::list<txPriorityNode>::iterator it=eventPool.begin(); it!=eventPool.end(); it++) {
		nodelist.push_back(&(*it));
	}

	std::sort(nodelist.begin(), nodelist.end(), txPriorityNodeCmp());
	eventQueue.clear();
	for (size_t i=0; i<nodelist.size(); i++) {
		eventQueue.push_back(nodelist[i]);
	}
}
