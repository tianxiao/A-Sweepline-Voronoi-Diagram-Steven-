#include <iostream.h>
#include "MapManager.h"

#define LOAD_GRID_MV_NAME "c:\\workarea\\testArea\\1_MVGridMap.mvm"
#define SAVE_GRID_MV_NAME "c:\\workarea\\testArea\\1_MVGridMap_result.mvm"

#define LOAD_GRID_POINTLIST_NAME "c:\\workarea\\testArea\\1_PointList.txt"
#define SAVE_GRID_POINTLIST_NAME "c:\\workarea\\testArea\\1_PointList_result.txt"

int testLoadSaveGrid(MapManager& manager);
int testGenerateDelaunay(MapManager* mgr);

//GET_SCREEN_cout_GLOBAL

int main()
{
	MapManager* mgr = new MapManager();

	testGenerateDelaunay(mgr);

	cout<<"\nAbout to clear the Delaunay"<<endl;

	mgr->clearDelaunay();

	cout<<"\nAbout to delete mgr"<<endl;

	delete mgr;
	mgr = 0;

	cout<<"After deleting the mgr, finishing"<<endl;

	return 0;
}

int testGenerateDelaunay(MapManager* mgr)
{
	
	mgr->loadMapViewerMap("c:\\test.mvm");

	mgr->generateDelaunay(0.5f,1.0f,3.0f);

	

	return 0;
}

int testLoadSaveGrid(MapManager& manager)
{
	bool ret = true;

	ret = manager.loadMapViewerMap(LOAD_GRID_MV_NAME);

	if(!ret) return 1;

	ret = manager.saveMapViewerMap(SAVE_GRID_MV_NAME,true,false);

	if(!ret) return 2;

	ret = SosUtil::compareFiles(LOAD_GRID_MV_NAME,SAVE_GRID_MV_NAME);

	if(!ret) return 3;

	ret = manager.loadMapAsPointList(LOAD_GRID_POINTLIST_NAME);

	if(!ret) return 4;

	ret = manager.saveMapAsPointList(SAVE_GRID_POINTLIST_NAME);

	if(!ret) return 5;
	
	ret = SosUtil::compareFiles(LOAD_GRID_POINTLIST_NAME,SAVE_GRID_POINTLIST_NAME);

	if(!ret) return 6;


	return 0;
}

