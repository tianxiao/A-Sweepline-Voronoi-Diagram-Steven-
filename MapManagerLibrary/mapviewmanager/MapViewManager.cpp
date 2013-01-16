/*
MapManager library for the conversion, manipulation and analysis 
of maps used in Mobile Robotics research.
Copyright (C) 2005 Shane O'Sullivan

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

email: shaneosullivan1@gmail.com
*/



#include "MapViewManager.h"

MapViewManager::MapViewManager(WindowCallbackInterface* parentWindow)
{
	_parentWindow = parentWindow;
	_mapManagerFullyOwned = true;
	_mapManager = new MapManager();

	init();
}

MapViewManager::MapViewManager(GridMap<float> *m,WindowCallbackInterface* parentWindow)
{
	_parentWindow = parentWindow;
	_mapManagerFullyOwned = true;
	_mapManager = new MapManager(m);

	init();
	addMap(m);
}

//this constructor is protected, so that only a child of this class
//can invoke it.  This is so that a sub class of this class
//can have an extended version of the MapManager class
//and have this, it's parent class, operate on it.
MapViewManager::MapViewManager(MapManager* childMapManager, WindowCallbackInterface* parentWindow)
{
	GET_FILE_LOG
	LOG<<"In MapViewManager::MapViewManager"<<endl;
	_parentWindow = parentWindow;
	_mapManagerFullyOwned = false;
	_mapManager = childMapManager;

	//init();
}

MapViewManager::MapViewManager(MapManager* childMapManager, GridMap<float> *m,WindowCallbackInterface* parentWindow)
{
	_parentWindow = parentWindow;
	_mapManagerFullyOwned = false;
	_mapManager = childMapManager;

	init();
	addMap(m);
}

MapViewManager::~MapViewManager()
{
	LOG<<"In MapViewManager::~MapViewManager()";
	if(_robotRunColours != 0)
	{
		delete _robotRunColours;
	}

	if(_mapManagerFullyOwned)
	{
		LOG<<"MapViewManager fully owns the map manager, so Deleting _mapManager"<<endl;
		delete _mapManager;
		_mapManager = 0;
	}
	else
	{
		LOG<<"Not deleting the _mapManager, it doesn't belong to this class";
	}
	LOG<<"At end of MapViewManager::~MapViewManager()";
}

void MapViewManager::init()
{	
	GET_FILE_LOG
	//LOGGING_OFF
	ENTRYEXIT_LOG_OFF

	LOG<<"In MapViewManager::init(), and _mapManager = "<<_mapManager<<endl;

	_mapManager->init();

	LOG<<"After calling _mapManager->init()"<<endl;
	_mapViewerBusy = false;

	_viewWidth= _viewHeight=0;
	_squareSize=4;
	_viewReduceFactor = 0;
	_gridDrawObject = 0;
	_drewBorder = false;
	_viewBorder = true;
	
	_vectorColour = 0;

	_randomMapOptions.avgRoomArea = 50;
	_randomMapOptions.createObstacles = true;
	_randomMapOptions.numMaps = 1;
	_randomMapOptions.numRobots = 1;
	_randomMapOptions.numRooms = 5;
	_randomMapOptions.percentObstacleArea = 10;
	_randomMapOptions.useBox = true;
	_randomMapOptions.useChair = false;
	_randomMapOptions.useDiningArea = false;
	_randomMapOptions.useRoomGridLayout = true;
	_sampleStep =1;

	_rulerObj = new VisRuler(this);

	enableRecordingChangedArea();

	_brushSize = 1;
	_oldviewWidth = -1;//this is used in the matchScreenToMap and unmatchScreenToMap methods
	_oldviewHeight = -1;//this is used in the matchScreenToMap and unmatchScreenToMap methods
	_rulerWasVisible = false;//this is used in the matchScreenToMap and unmatchScreenToMap methods

	//when a path is being planned, if this variable, _layerWhenLastPathPlanned, is different from
	//the latest layer, a new copy of the map is made.  Otherwise, no change has been made to the map,
	//so the old copy is used
	_layerWhenLastPathPlanned = -1;

	_voronoiColour = RGB(50,50,250); //voronoi lines are blue
	_pathColour = RGB(50,200,50);
	_robotColour = RGB(50,150,10);

	_delaunayColour = RGB(10,130,10);//delaunay lines are greenish

	_clearedBackground = false;

	int i = 0;
	for(i = 0; i<256; i++)
	{
		_gridColours[i] = RGB(i,i,i);
	}

	_mapAverageCount = 1;

	_robotRadius = 1;
	
	_currentX = 0;
	_currentY = 0;
	_iteratorRightX=_iteratorLeftX=_iteratorTopY=_iteratorBottomY=0;

	//booleans used for speeding up drawing
	//all areas outside the map will have the same area, 
	//so draw them using 4 big rectangles
	_drawOutSideMap=false;

	_getMapFromView = false;

	_viewVoronoi			= false;
	_viewVoronoiVertices	= false;
	_viewBackgroundGrid		= true;
	_viewPath = false;
	_viewRobotRun = false;

	setViewAvgCellValue();

	_finishedDrawingGrid = false;
	_finishedDrawingBackgroundGrid = false;
	_finishedDrawingRuler = true;
	_finishedDrawingPath = true;
	_finishedDrawingVoronoi = true;
	_finishedDrawingVoronoiVertices = true;
	_finishedDrawingRobotRun = true;

	_getNextRobotRunList = false;

	_currentRobotRunList = 0;

	//these three options can be defined by the user, and specify how an A-Star path is created

	_robotRunColours = new Vector<DWORD>((DWORD)0);

	int colourSize = MAX_NUM_ROBOT_RUNS;
	DWORD colours[MAX_NUM_ROBOT_RUNS] = { 
						RGB(0,250,0),	RGB(0,0,250),	RGB(250,0,0),	
						RGB(0,250,250),	RGB(250,250,0),	RGB(250,0,250),	
						RGB(0,0,100),	RGB(0,100,0),	RGB(100,0,0),	
						RGB(0,100,100),	RGB(100,100,0),	RGB(100,0,100),
						RGB(0,0,50),	RGB(0,50,0),	RGB(50,0,0),
						RGB(0,50,50),	RGB(50,50,0),	RGB(50,0,50),
						RGB(0,0,175),	RGB(0,175,0),	RGB(175,0,0),
						RGB(0,175,175),	RGB(175,175,0),	RGB(175,0,175),
						};

	for(i = 0; i< colourSize; i++)
	{
		_robotRunColours->push_back(colours[i]);
	}

	PointXYLong pt(0,0);
	_currentDrawingRobot.setRadiusAndCentre(pt,5);
	_currentDrawingRobot.setOrientation(25);
	_currentDrawingRobot.setVisible(_viewRobot);

	LOG<<"At the end of MapViewManager::init()"<<endl;
}

void MapViewManager::resetAllObjects()
{
	LOG<<"In MapViewManager::resetAllObjects()";
	if(_mapManager != 0)
	{
		_mapManager->resetAllObjects();
	}
	_listGridBackground.clear();
	_layerWhenLastPathPlanned = -1;

	delete _rulerObj;
	_rulerObj = new VisRuler(this);

	LOG<<"At end of MapViewManager::resetAllObjects()";

}

int MapViewManager::addMap(GridMap<float> *m, bool performShallowCopy)
{
	//if the map is not null, initialise the view to look at the complete map
	if(m != 0)
	{
		_mapManager->addMap(m,performShallowCopy);
		resetView();

		calcSquareSize();

		_finishedDrawingRuler = false;
		setAllScreenToChanged();
	}

	_mapAverageCount = 1;

	LOGEXIT("addMap")
	return 0;
}


bool MapViewManager::newMap(long minX, long maxX, long minY, long maxY)
{
	_mapManager->newMap(minX,maxX,minY,maxY);
	resetView();
	return true;
}

bool MapViewManager::loadSaphiraWld(char* filePath)
{
	if(filePath == 0)
	{
		_parentWindow->popupOk(STD_ERROR_TITLE, ERROR_FILE_OPEN(filePath));
		return false;
	}
	bool retval = _mapManager->loadSaphiraWld(filePath);

	if(retval)
	{
		setViewRobot(true);
		resetView();		
		_parentWindow->updateToolbarButtons();
	}

	return retval;
}


bool MapViewManager::loadVoronoi(char* filePath)
{
	if(filePath == 0)
	{
		_parentWindow->popupOk(STD_ERROR_TITLE, ERROR_FILE_OPEN(filePath));
		return false;
	}
	
	bool retval = _mapManager->loadVoronoi(filePath);

	if(retval)
	{
		resetView();

		if(_mapManager->hasDelaunay())
		{
			setViewDelaunay(true);
		}
		if(_mapManager->hasVoronoi())
		{
			setViewVoronoi(true);
		}

		_parentWindow->updateToolbarButtons();
	}

	return retval;
}

//Loads a map as a list of points.  The format is as follows:
//
//gridpointlist
//width 154
//height 203
//0 0 0.5
//0 1 0.6 
//......etc 
bool MapViewManager::loadMapAsPointList(char* filePath)
{
	if(filePath == 0)
	{
		_parentWindow->popupOk(STD_ERROR_TITLE, ERROR_FILE_OPEN(filePath));
		return false;
	}
	
	bool retval = _mapManager->loadMapAsPointList(filePath);

	if(retval)
	{
		resetView();
		_parentWindow->updateToolbarButtons();
	}
	return retval;

}

bool MapViewManager::loadGridMap(char* filePath)
{
	if(filePath == 0)
	{
		_parentWindow->popupOk(STD_ERROR_TITLE, ERROR_FILE_OPEN(filePath));
		return false;
	}
	
	bool retval = _mapManager->loadGridMap(filePath);

	if(retval)
	{
		resetView();
		_parentWindow->updateToolbarButtons();
	}

	return retval;
}

bool MapViewManager::loadCarmen(char* filePath)
{
	if(filePath == 0)
	{
		_parentWindow->popupOk(STD_ERROR_TITLE, ERROR_FILE_OPEN(filePath));
		return false;
	}
	
	bool retval = _mapManager->loadCarmen(filePath);

	if(retval)
	{
		resetView();
		_parentWindow->updateToolbarButtons();
	}

	return retval;
}

bool MapViewManager::loadBeeSoft(char* filePath)
{
	if(filePath == 0)
	{
		_parentWindow->popupOk(STD_ERROR_TITLE, ERROR_FILE_OPEN(filePath));
		return false;
	}
	
	bool retval = _mapManager->loadBeeSoft(filePath);

	if(retval)
	{
		resetView();
		_parentWindow->updateToolbarButtons();
	}

	return retval;
}


bool MapViewManager::loadStageMap(char* filePath)
{
	if(filePath == 0)
	{
		_parentWindow->popupOk(STD_ERROR_TITLE, ERROR_FILE_OPEN(filePath));
		return false;
	}
	
	bool retval = _mapManager->loadStageMap(filePath);

	if(retval)
	{
		setViewRobot(true);
		resetView();
		_parentWindow->updateToolbarButtons();
	}

	return retval;
}

bool MapViewManager::saveStage(char* filePath, StageWorldOptions& options)
{	
	if(filePath == 0)
	{
		_parentWindow->popupOk(STD_ERROR_TITLE, ERROR_FILE_OPEN(filePath));
		return false;
	}
	
	float threshold = 0;
	if(!_parentWindow->getNumber1("Please enter occupancy threshold for PNM image", false,0.5f,0.1f,1.0f,threshold))
	{
		return false;
	}

	bool retval = _mapManager->saveStage(filePath,options,threshold);

	if(retval)
	{
		resetView();
		_parentWindow->updateToolbarButtons();
	}

	return retval;
}


bool MapViewManager::loadMapViewerMap(char* filePath)
{
	if(filePath == 0)
	{
		_parentWindow->popupOk(STD_ERROR_TITLE, ERROR_FILE_OPEN(filePath));
		return false;
	}
	
	bool retval = _mapManager->loadMapViewerMap(filePath);

	if(retval)
	{
		if(_mapManager->hasRobot())
			setViewRobot(true);

		resetView();
		_parentWindow->updateToolbarButtons();
	}

	return retval;
}

bool MapViewManager::loadPath(char* filePath)
{
	if(filePath == 0)
	{
		_parentWindow->popupOk(STD_ERROR_TITLE, ERROR_FILE_OPEN(filePath));
		return false;
	}

	bool retval = _mapManager->loadPath(filePath);

	if(retval)
	{
		resetView();
		_parentWindow->updateToolbarButtons();
		setViewPath(true);
	}

	return retval;
}



bool MapViewManager::loadRobotRun(char* filePath)
{
	if(_listRobotRunLists.getListSize() >= MAX_NUM_ROBOT_RUNS)
	{
		_parentWindow->popupOk(STD_ERROR_TITLE, ERROR_REACHED_MAX("robot runs"));		
		return false;
	}

	if(filePath == 0)
	{
		_parentWindow->popupOk(STD_ERROR_TITLE, ERROR_FILE_OPEN(filePath));
		return false;
	}	

	bool retval = _mapManager->loadMapViewerMap(filePath);

	if(retval)
	{
		resetView();
		_parentWindow->updateToolbarButtons();
		setViewRobotRun(true);
	}

	return retval;
}


void MapViewManager::refresh()
{
	calcSquareSize();	
}

void MapViewManager::resetView()
{
	LOGENTRY("resetView()")

	_mapManager->getDimensions(_mapMinX,_mapMaxY,_mapMaxX,_mapMinY);

	float x = 0, y = 0;
	
	_mapManager->mmToGrid(_mapMinX,_mapMaxY,x,y);
	_mapMinX = (long)x;
	_mapMaxY = (long)y;
	
	_mapManager->mmToGrid(_mapMaxX,_mapMinY,x,y);
	_mapMaxX = (long)x;
	_mapMinY = (long)y;

	_changedValuesUnset = false;
	setAllScreenToChanged();
	calcSquareSize();
	
	LOGEXIT("resetView()")
}


void MapViewManager::setAllScreenToChanged()
{
	LineXYLong currentMapSize ;
	_mapManager->getDimensions(currentMapSize.pt1.x,currentMapSize.pt1.y,currentMapSize.pt2.x,currentMapSize.pt2.y);

	long minX,maxX,minY,maxY;
	minX = maxX = minY = maxX = 0;

	LOG<<"setAllScreenToChanged mapMinX = "<<_mapMinX<<", _mapMaxX = "<<_mapMaxX<<", _mapMinY = "<<_mapMinY<<", _mapMaxY = "<<_mapMaxY;

	_mapManager->gridToMm(_mapMaxX,_mapMaxY,maxX,maxY);
	_mapManager->gridToMm(_mapMinX,_mapMinY,minX,minY);

	addChangedArea(maxX,maxY,minX,minY,currentMapSize);
	_changedValuesUnset = false;

	//if the ruler is visible, set it up with the correct values
	if(_rulerObj->isVisible())
	{
		_finishedDrawingRuler = false;
	}
}

void MapViewManager::setScreenAreaToChanged(long x1,long y1,long x2,long y2, int expand)
{
	LineXYLong currentMapSize ;
	_mapManager->getDimensions(currentMapSize.pt1.x,currentMapSize.pt1.y,currentMapSize.pt2.x,currentMapSize.pt2.y);
	
	long x1f = 0,y1f = 0,x2f = 0,y2f = 0;
	long height = getDisplayedMapHeight(), width = getDisplayedMapWidth();
	SosUtil::ensureSmaller(x1,x2);
	SosUtil::ensureSmaller(y1,y2);
	int res = _mapManager->getResolution();
	x1 = (x1 > expand) ? x1 - expand * res: 1;
	y1 = (y1 > expand) ? y1 - expand * res: 1;
	x2 = (x2 < width - expand) ? x2 + expand * res: width;
	y2 = (y2 < height - expand) ? y2 + expand * res: height;

	screenToMm(x1,y1,x1f,y1f);
	screenToMm(x2,y2,x2f,y2f);
	addChangedArea(x1f,y1f,x2f,y2f,currentMapSize);

	_changedValuesUnset = false;
}

void MapViewManager::addChangedArea(float x1, float y1, float x2, float y2, LineXYLong prevArea)
{
	if(!_changedAreaEnabled)
		return;
	x1 = (x1 < 0) ? x1 - 1 : x1;
	y1 = (y1 < 0) ? y1 - 1 : y1;
	x2 = (x2 < 0) ? x2 - 1 : x2;
	y2 = (y2 < 0) ? y2 - 1 : y2;
	addChangedArea((long)x1,(long)y1,(long)x2,(long)y2,prevArea);
}

void MapViewManager::addChangedArea(long x1, long y1, long x2, long y2, LineXYLong prevArea)
{
	if(!_changedAreaEnabled)
		return;

	SosUtil::ensureSmaller(x1,x2);
	SosUtil::ensureSmaller(y1,y2);

	long west = 0, east = 0, north = 0, south = 0;
	west = prevArea.pt1.x;
	east = prevArea.pt2.x;
	south = prevArea.pt2.y;
	north = prevArea.pt1.y;

	SosUtil::ensureSmaller(south,north);
	SosUtil::ensureSmaller(west,east);

	if(x1 < west )
	{
		y1 = SosUtil::minVal(south, y1);
		y2 = SosUtil::maxVal(north, y2);
		x2 = SosUtil::maxVal(west-1,x2);
	}
	if(x2 > east)
	{
		y1 = SosUtil::minVal(south, y1);
		y2 = SosUtil::maxVal(north, y2);
		x1 = SosUtil::minVal(east,x1);
	}
	if(y1 < south)
	{
		x1 = SosUtil::minVal(x1,west);
		x2 = SosUtil::maxVal(x2, east);
		y2 = SosUtil::maxVal(y2,south-1);
	}
	if(y2 > north)
	{
		x1 = SosUtil::minVal(x1,west);
		x2 = SosUtil::maxVal(x2, east);
		y1 = SosUtil::minVal(y1,north+1);		
	}	

	if(_changedValuesUnset)
	{
		_changedMaxX = x2;
		_changedMinX = x1;
		_changedMinY = y1;
		_changedMaxY = y2;
		_changedValuesUnset = false;
	}
	else
	{	
		_changedMaxX = SosUtil::maxVal(_changedMaxX,x2);
		_changedMinX = SosUtil::minVal(_changedMinX,x1);
		_changedMinY = SosUtil::minVal(_changedMinY,y1);
		_changedMaxY = SosUtil::maxVal(_changedMaxY,y2);
	}
	LOG<<"addChangedArea() minX = "<<_changedMinX<<", maxX = "<<_changedMaxX<<", minY = "<<_changedMinY<<", maxY = "<<_changedMaxY;
}

bool MapViewManager::getMapCoordsSet(long viewX, long viewY, long& mapX1, long& mapY1, long& mapX2, long& mapY2)
{
	long tempX1 = 0, tempY1 = 0, tempX2 = 0, tempY2 = 0;
	bool retval = false;

	//if we are not drawing the map using a view, then any point on the screen can only be inside
	//a single map cell, so the getMapCoords funciton should really have been called
	if(!_getMapFromView)
	{
		retval = getMapCoords(viewX,viewY,tempX1,tempY1);
		if(retval)
		{
			mapX1 = mapX2 = tempX1;
			mapY1 = mapY2 = tempY1;
		}
		return retval;
	}
	else
	{		
		if(_squareSize != 0)
		{	
//			LOG<<"getMapCoordsSet got viewX = "<<viewX<<", viewY = "<<viewY;
//			LOG<<"_squareSize = "<<_squareSize<<", _viewReduceFactor = "<<_viewReduceFactor;
			viewX /= _squareSize;//this will tell us how many blocks from the left the point is
			viewY /= _squareSize;//this will tell us how many blocks from the top the point is
	
			viewX *= _viewReduceFactor;
			viewY *= _viewReduceFactor;

			mapX1 = _mapMinX + viewX;
			mapX2 = _mapMinX + viewX + _viewReduceFactor -1;

			mapY1 = _mapMaxY -viewY;
			mapY2 = _mapMaxY -viewY - _viewReduceFactor + 1;

	//		LOG<<"returning minX = "<<mapX1<<", maxX = "<<mapX2;

			//if the requested point is outside the map, return false.  The values in the variables are still
			//changed however
			if(mapX1 < _mapMinX || mapX1 > _mapMaxX || mapY1 < _mapMinY || mapY1 > _mapMaxY
				|| mapX2 < _mapMinX || mapX2 > _mapMaxX || mapY2 < _mapMinY || mapY2 > _mapMaxY)
			{
				return false;
			}
			else
			{
				return true;
			}
		}
		else
		{
			return false;
		}
	}
}


bool MapViewManager::getMapCoords(long viewX, long viewY, long& mapX, long& mapY)
{
//	LOG<<"MapViewManager::getMapCoords("<<viewX<<","<<viewY<<")";

	long mapX2 = 0, mapY2 = 0;

	float tempX1 = 0, tempY1 = 0;
	//if the map is not too big to fit easily in the window, then calculate the map coordinates
	//the simple way
	if(!_getMapFromView )
	{		
		if(_squareSize != 0)
		{
			screenToGrid(viewX,viewY,tempX1,tempY1);
			mapX = (long)tempX1;
			mapY = (long)tempY1;
			LOG<<"getMapCoords ("<<viewX<<","<<viewY<<") -> ("<<mapX<<","<<mapY<<")";
			//viewX /= _squareSize;
			//viewY /= _squareSize;

			//mapX = _mapMinX + viewX;
			//mapY = _mapMaxY - viewY;
		
			//if the requested point is outside the map, return false.  The values in the variables are still
			//changed however
			if(mapX < _mapMinX || mapX > _mapMaxX || mapY < _mapMinY || mapY > _mapMaxY)
			{
				return false;
			}
			else
			{
				return true;
			}
		}
		return false;
	}
	else
	{
		getMapCoordsSet(viewX,viewY,mapX,mapY,mapX2,mapY2);

		return true;

	}
	return true;
}

inline bool MapViewManager::getViewAreaFromMap(long mapX, long mapY, long& viewX1,long& viewY1,long& viewX2,long& viewY2)
{
	bool objectCompletelyInViewWindow = true;
	if(mapX < _mapMinX || mapX > _mapMaxX || mapY < _mapMinY || mapY >_mapMaxY)
	{
		objectCompletelyInViewWindow = false;
	}

	if(!_getMapFromView)
	{
		mapX -= _mapMinX;
		mapY = _mapMaxY - mapY;

		viewX1 = mapX * _squareSize;
		viewY1 = mapY * _squareSize;

		viewX2 = (viewX1) + _squareSize-1;
		viewY2 = (viewY1) + _squareSize-1;	
	}
	else
	{		
		mapX -= _mapMinX;
		mapY = _mapMaxY - mapY;
		
		mapX /= _viewReduceFactor;
		mapY /= _viewReduceFactor;
		
		viewX1 = mapX * _squareSize;
		viewY1 = mapY * _squareSize;

		viewX2 = viewX1 + _squareSize -1;
		viewY2 = viewY1 + _squareSize -1;			
	}
	
	return objectCompletelyInViewWindow;
}
inline void MapViewManager::getViewAreaFromMapStandardView(long mapX, long mapY, long& viewX1,long& viewY1,long& viewX2,long& viewY2)
{
	mapX -= _mapMinX;
	mapY = _mapMaxY - mapY;

	viewX1 = mapX * _squareSize;
	viewY1 = mapY * _squareSize;

	viewX2 = (viewX1) + _squareSize-1;
	viewY2 = (viewY1) + _squareSize-1;
}
inline void MapViewManager::getViewAreaFromMapUsingView(long mapX, long mapY, long& viewX1,long& viewY1,long& viewX2,long& viewY2)
{
	mapX -= _mapMinX;
	mapY = _mapMaxY - mapY;
	
	mapX /= _viewReduceFactor;
	mapY /= _viewReduceFactor;
	
	viewX1 = mapX * _squareSize;
	viewY1 = mapY * _squareSize;

	viewX2 = viewX1 + _squareSize -1;
	viewY2 = viewY1 + _squareSize -1;
}


//this version of getViewAreaFromMap takes two map points, and returns the
//screen rectangle that surrounds them, unlike the other version, which just does one 
//point
bool MapViewManager::getViewAreaFromMap(long mapX1, long mapY1, long mapX2, long mapY2,
										long& viewX1,long& viewY1,long& viewX2,long& viewY2)
{
	bool objectCompletelyInViewWindow = true;
	SosUtil::ensureSmaller(mapX1,mapX2);
	SosUtil::ensureSmaller(mapY1,mapY2);//(x1, y1) is the bottom left corner, (x2,y2) is top right corner

	if(mapX1 < _mapMinX || mapX2 > _mapMaxX || mapY1 < _mapMinY || mapY2 >_mapMaxY)
	{
		objectCompletelyInViewWindow = false;
	}

	if(!_getMapFromView)
	{
		mapX1 -= _mapMinX;
		mapY1 = _mapMaxY - mapY1;
		mapX2 -= _mapMinX;
		mapY2 = _mapMaxY - mapY2;


		viewX1 = mapX1 * _squareSize;
		viewY1 = mapY1 * _squareSize;

		viewX2 = (mapX2 * _squareSize) + _squareSize;
		viewY2 = (mapY2 * _squareSize) + _squareSize;	
	}
	else
	{				
		mapX1 -= _mapMinX;
		mapY1 = _mapMaxY - mapY1;
		mapX2 -= _mapMinX;
		mapY2 = _mapMaxY - mapY2;
		
		mapX1 /= _viewReduceFactor;
		mapY1 /= _viewReduceFactor;
		mapX2 /= _viewReduceFactor;
		mapY2 /= _viewReduceFactor;
		
		viewX1 = mapX1 * _squareSize;
		viewY1 = mapY1 * _squareSize;

		viewX2 = (mapX2 * _squareSize) + _squareSize;
		viewY2 = (mapY2 * _squareSize) + _squareSize;					
	}
	
	return objectCompletelyInViewWindow;
}
//this version of getViewAreaFromMap takes two map points, and returns the
//screen rectangle that surrounds them, unlike the other version, which just does one 
//point
inline void MapViewManager::getViewAreaFromMapStandardView(long mapX1, long mapY1, long mapX2, long mapY2,
										long& viewX1,long& viewY1,long& viewX2,long& viewY2)
{
	SosUtil::ensureSmaller(mapX1,mapX2);
	SosUtil::ensureSmaller(mapY1,mapY2);//(x1, y1) is the bottom left corner, (x2,y2) is top right corner

	mapX1 -= _mapMinX;
	mapY1 = _mapMaxY - mapY1;
	mapX2 -= _mapMinX;
	mapY2 = _mapMaxY - mapY2;


	viewX1 = mapX1 * _squareSize;
	viewY1 = mapY1 * _squareSize;

	viewX2 = (mapX2 * _squareSize) + _squareSize;
	viewY2 = (mapY2 * _squareSize) + _squareSize;	

}

//this version of getViewAreaFromMap takes two map points, and returns the
//screen rectangle that surrounds them, unlike the other version, which just does one 
//point
void MapViewManager::getViewAreaFromMapUsingView(long mapX1, long mapY1, long mapX2, long mapY2,
										long& viewX1,long& viewY1,long& viewX2,long& viewY2)
{
	SosUtil::ensureSmaller(mapX1,mapX2);
	SosUtil::ensureSmaller(mapY1,mapY2);//(x1, y1) is the bottom left corner, (x2,y2) is top right corner

				
	mapX1 -= _mapMinX;
	mapY1 = _mapMaxY - mapY1;
	mapX2 -= _mapMinX;
	mapY2 = _mapMaxY - mapY2;
	
	mapX1 /= _viewReduceFactor;
	mapY1 /= _viewReduceFactor;
	mapX2 /= _viewReduceFactor;
	mapY2 /= _viewReduceFactor;
	
	viewX1 = mapX1 * _squareSize;
	viewY1 = mapY1 * _squareSize;

	viewX2 = (mapX2 * _squareSize) + _squareSize;
	viewY2 = (mapY2 * _squareSize) + _squareSize;				
}

bool MapViewManager::pointInViewArea(long viewX, long viewY)
{
	if(viewX > getDisplayedMapWidth() || viewX < 0
		|| viewY > getDisplayedMapHeight() || viewY < 0)
	{
		return false;
	}

	return true;

}

void MapViewManager::zoomIn(long minX, long minY, long maxX, long maxY)
{
	long temp = 0;

	LOG<<"MapViewManager::zoomIn("<<minX<<","<<minY<<","<<maxX<<","<<maxY<<")"<<endl;

	SosUtil::ensureSmaller(minX,maxX);
	SosUtil::ensureSmaller(maxY,minY);
	
	long tempMinX = 0, tempMinY = 0, tempMaxX = 0, tempMaxY = 0;

	if(!_getMapFromView)
	{
		//if either of the map coordinates is invalid, do nothing
		if(!getMapCoords(minX,minY,tempMinX, tempMinY) || 
			!getMapCoords(maxX, maxY, tempMaxX, tempMaxY))
		{
			LOG<<"zoomIn(): Error, doing nothing because one of the two "	<<"points is invalid";
			return;
		}
	}
	else
	{ 
		//since a single point can represent a number of cells when zoomed out
		//get the top left and bottom right range of each with getMapCoordsSet
		long x1, y1 , x2 , y2;

		x1 = y1 = x2 = y2 = 0;
		getMapCoordsSet(minX, minY, x1, y1 , x2 , y2);
		tempMinX = SosUtil::minVal(x1,x2);
		tempMinY = SosUtil::minVal(y1,y2);

		x1 = y1 = x2 = y2 = 0;
		getMapCoordsSet(maxX, maxY, x1, y1 , x2 , y2);
		tempMaxX = SosUtil::maxVal(x1,x2);
		tempMaxY = SosUtil::maxVal(y1,y2);

	}
	LOG<<"zoomIn(): minX = "<<tempMinX<<", minY = "<<tempMinY<<", maxX = "<<tempMaxX<<", maxY = "<<tempMaxY;

	_mapMinX = tempMinX;
	_mapMinY = tempMinY;
	_mapMaxX = tempMaxX;
	_mapMaxY = tempMaxY;

	calcSquareSize();	

	_displayedArea.x = getDisplayedMapWidth();
	_displayedArea.y = getDisplayedMapHeight();
	setAllScreenToChanged();
}

bool MapViewManager::zoomOut(double percentage)
{
	LOG<<"zoomOut() entry::";

	if(!_mapViewerBusy)
	{
		_mapViewerBusy = true;
	}
	else
	{
		return false;
	}

	double diffX = 0;
	double diffY = 0;
	double midX = 0;
	double midY = 0;

	diffX = ((_mapMaxX - _mapMinX)/2);
	diffY = ((_mapMaxY - _mapMinY)/2);
	midX = _mapMinX + diffX;
	midY = _mapMinY + diffY;

	percentage = 1 + (percentage/100);

	LOG<<"zoomOut():: orig map display values->:_mapMaxX = "<<_mapMaxX<<", _mapMinX = "<<_mapMinX<<", _mapMaxY = "<<_mapMaxY<<" , _mapMinY = "<<_mapMinY;

	_mapMaxX = SosUtil::maxVal(long(midX + (diffX * percentage)),_mapMaxX+1);
	_mapMinX = SosUtil::minVal(long(midX - (diffX * percentage)),_mapMinX - 1);
	_mapMaxY = SosUtil::maxVal(long(midY + (diffY * percentage)),_mapMaxY + 1);
	_mapMinY = SosUtil::minVal(long(midY - (diffY * percentage)),_mapMinY - 1);
	LOG<<"zoomOut():: new map display values->:_mapMaxX = "<<_mapMaxX<<", _mapMinX = "<<_mapMinX	<<", _mapMaxY = "<<_mapMaxY<<" , _mapMinY = "<<_mapMinY;

	calcSquareSize();
	_displayedArea.x = getDisplayedMapWidth();
	_displayedArea.y = getDisplayedMapHeight();

	_mapViewerBusy = false;

	setAllScreenToChanged();
	return true;

	LOG<<"zoomOut() exit::";

}

void MapViewManager::setDisplayedMapArea(int direction, long value)
{
	switch(direction)
	{
	case NORTH:
		_mapMaxY = value;
		break;
	case SOUTH:
		_mapMinY = value;
		break;
	case EAST:
		_mapMaxX = value;
		break;
	case WEST:
		_mapMinX = value;
		break;			
	}
	calcSquareSize();	
}

bool MapViewManager::resizeWindow(long newWidth, long newHeight)
{
	LOGENTRY("resizeWindow");
	LOG<<"resizeWindow entry, newWidth = "<<newWidth<<" , newHeight = "<<newHeight<<endl;
	_viewWidth = newWidth;
	_viewHeight = newHeight;
	
	bool retval = calcSquareSize();

	LOGEXIT("resizeWindow");

	return retval;
}

PointXYLong MapViewManager::getWindowSize()
{
	PointXYLong ret;
	ret.x = _viewWidth;
	ret.y = _viewHeight;
	return ret;


}
//
bool MapViewManager::calcSquareSize()
{
	LOGENTRY("calcSquareSize()");

	bool useHeight = false;
	long origSqSize = _squareSize, origViewReduceFactor = _viewReduceFactor;

	if(_viewHeight <= 0 || _viewWidth <= 0)
		return false;
	
	long viewHeightUsed = _viewHeight;
	long _viewWidthUsed = _viewWidth;
	//if the ruler is visible, make room for it in the view window
	if(_rulerObj->isVisible())
	{
		viewHeightUsed -= _rulerObj->getBarWidth();
		_viewWidthUsed -= _rulerObj->getBarWidth();
	}

	_viewReduceFactor = 1;

	if((_mapMaxY - _mapMinY + 1) != 0 && (_mapMaxX - _mapMinX + 1)!=0)
	{
		if(double(viewHeightUsed)/double(_mapMaxY - _mapMinY + 1) < double(_viewWidthUsed)/double(_mapMaxX - _mapMinX + 1))
		{
			_squareSize = viewHeightUsed/(_mapMaxY - _mapMinY + 1);

			useHeight = true;
		}
		else
		{
			_squareSize = _viewWidthUsed/(_mapMaxX - _mapMinX + 1);
			useHeight = false;
		} 

		if(_squareSize < MIN_SQUARE_SIZE)
		{
			_getMapFromView = true;
			
			_viewReduceFactor = 2;

			if(useHeight && viewHeightUsed != 0)
			{
				while(double(viewHeightUsed)/double(_mapMaxY - _mapMinY + 1)*_viewReduceFactor < MIN_SQUARE_SIZE 
					&& _viewReduceFactor < MAX_VIEW_REDUCTION)
				{
					_viewReduceFactor++;
				}

				_squareSize = long((double(viewHeightUsed)/double(_mapMaxY - _mapMinY + 1))*_viewReduceFactor);
			}
			else if(_viewWidthUsed != 0)
			{
				while(double(_viewWidthUsed)/double(_mapMaxX - _mapMinX + 1)*_viewReduceFactor < MIN_SQUARE_SIZE
					&& _viewReduceFactor < MAX_VIEW_REDUCTION)
				{
					_viewReduceFactor++;	
				}
				_squareSize = long((double(_viewWidth)/double(_mapMaxX - _mapMinX + 1))*_viewReduceFactor);
			}

			if(_viewReduceFactor >= MAX_VIEW_REDUCTION)
			{
				resetView();
				return true;
			}
	

			//this is how many cells to skip when condensing a map to fit the screen.
			_sampleStep = SosUtil::maxVal(long(SosUtil::roundUp(double(_viewReduceFactor)/double(MAX_MAP_REDUCTION_SIZE))),(long)1);
			
		}
		else
		{
			_getMapFromView = false;

			//this is how many cells to skip when condensing a map to fit the screen. 1 means none are skipped
			_sampleStep = 1; 
		}
	}
	else
	{
		_squareSize = 0;
	}

	_displayedArea = getViewSize();

	//SET UP THE GRID IN THE BACKGROUND

	_listGridBackground.clear();
	
	long topY = 0; //the one background grid cell takes up 10 cells -> might have to change this
	long rightX = 0, leftX = 0, bottomY = 0;
	LineXYLayer line;
	long minX = 0, minY = 0, maxX = 0, maxY = 0;

	//only draw the grid if the are actually spaces between the lines - otherwise just draw one 
	//filled rectangle covering everything - which is what would happen if each line were
	//1 pixel apart, only drawing the rectangle is much quicker
	if(_viewReduceFactor < 10)
	{
		_mapManager->gridToMm(_mapMaxX - (_mapMaxX % 10), _mapMinY, rightX,bottomY);
		_mapManager->gridToMm(_mapMinX, _mapMaxY - (_mapMaxY % 10), leftX,topY);

		_mapManager->gridToMm(_mapMinX,_mapMinY,minX,minY);
		_mapManager->gridToMm(_mapMaxX,_mapMaxY,maxX,maxY);
				
		line.type = OBJECT_TYPE_LINE;
		line.pt1.y= minY;
		line.pt2.y = maxY+_mapManager->getResolution();

		for(long x = rightX; x > leftX; x -= 10 * _mapManager->getResolution())
		{
			line.pt1.x = (float)x;
			line.pt2.x = (float)x;
			_listGridBackground.push(line);
		}

		line.pt1.x= minX;
		line.pt2.x = maxX + _mapManager->getResolution();

		for(int y = topY; y > bottomY; y-=10 * _mapManager->getResolution())
		{
			line.pt1.y = (float)y;
			line.pt2.y = (float)y;
			_listGridBackground.push(line);
		}
	}
	else
	{
		line.type = OBJECT_TYPE_RECTANGLE_FILLED;
		_mapManager->gridToMm(_mapMinX, _mapMinY,minX,minY);
		_mapManager->gridToMm(_mapMaxX, _mapMaxY,maxX,maxY);

		line.pt1.x = (float)minX;
		line.pt1.y = (float)minY;
		line.pt2.x = (float)maxX;
		line.pt2.y = (float)maxY;
		
		_listGridBackground.push(line);

	}

	//SET UP THE RULER
	//if the ruler is visible, set it up with the correct values
	if(_rulerObj->isVisible())
	{
		_finishedDrawingRuler = false;
	}

	LOG<<"calcSquareSize mapMinX = "<<_mapMinX<<", _mapMaxX = "<<_mapMaxX<<", _mapMinY = "<<_mapMinY<<", _mapMaxY = "<<_mapMaxY;

	
	_rulerObj->setResolution((float)_mapManager->getResolution());//display in metres
	//set the bottom right corner that the ruler will be at - the last 2 parameters are ignored
	_rulerObj->setPoints(_displayedArea.x + _rulerObj->getBarWidth(), _displayedArea.y + _rulerObj->getBarWidth(),0,0);
	_rulerObj->setNumDivisions(_mapMinX,_mapMaxX,_mapMinY,_mapMaxY, _squareSize);

	setAllScreenToChanged();

	return true; //if the view has changed because of the resize, return true. Otherwise return false

}

bool MapViewManager::resetIterator()
{	
	//reset the display iterator to display the area of the map currenty in the view area
	return resetIterator(0,0,getDisplayedMapWidth(), getDisplayedMapHeight());	
}

LineXYLong MapViewManager::getChangedArea()
{
	LineXYLong line(0,0,0,0);
	if(_changedValuesUnset)
	{
		LOG<<"_changedValuesUnset = true";
		return line;
	}

	long x1 = 0, y1 = 0, x2 = 0, y2 = 0;
	PointXYLong pt1, pt2;
	const long zero = 0;
	bool retval = false;


	mmToScreen((float)_changedMinX,(float)_changedMaxY+1,x1,y1);

	mmToScreen((float)_changedMaxX+1,(float)_changedMinY-1,x2,y2);

	LOG<<"getChangedAread() _squareSize = "<<_squareSize;
	LOG<<"getChangedAread() minX = "<<_changedMinX<<", maxX = "<<_changedMaxX<<", minY = "<<_changedMinY<<", maxY = "<<_changedMaxY;
	

	pt1.x = SosUtil::maxVal(SosUtil::minVal(x1,x2)-1,zero);
	pt1.y = SosUtil::maxVal(SosUtil::minVal(y1,y2)-1,zero);

	pt2.x = SosUtil::minVal(SosUtil::maxVal(x1,x2)+1,getDisplayedMapWidth());
	pt2.y = SosUtil::minVal(SosUtil::maxVal(y1,y2)+1,getDisplayedMapHeight());

	line.pt1 = pt1;
	line.pt2 = pt2;

	LOG<<"getChangedArea returning "<<line;

	return line;

}

bool MapViewManager::resetIteratorOnlyChanged(int expand)
{
	LOG<<"In resetIteratorOnlyChanged";
	//if nothing has changed, do nothing
	if(_changedValuesUnset || _mapViewerBusy)
	{
		LOG<<"resetIteratorOnlyChanged returning false because _changedValuesUnset = "<<_changedValuesUnset<<" and _mapViewerBusy = "<<_mapViewerBusy;
		return false;
	}

	expand *= _viewReduceFactor;

	long x1 = 0, y1 = 0, x2 = 0, y2 = 0;
	PointXYLong pt1, pt2;
	float minX, maxX,minY,maxY;
	minX = maxX = minY = maxY = 0;

	LOG<<"_changedMinX = "<<_changedMinX<<", _changedMaxX = "<<_changedMaxX<<", _changedMaxY = "<<_changedMaxY<<", _changedMinY = "<<_changedMinY;
	LOG<<"_mapMinX = "<<_mapMinX<<", _mapMaxX = "<<_mapMaxX<<", _mapMinY = "<<_mapMinY<<", _mapMaxY = "<<_mapMaxY;

	
	_mapManager->mmToGrid(_changedMinX, _changedMaxY, minX,maxY);
	_mapManager->mmToGrid(_changedMaxX, _changedMinY, maxX,minY);
	
	getViewAreaFromMap(minX - expand,maxY + expand,x1,y1,x2,y2);

	pt1.x = SosUtil::minVal(x1,x2);
	pt1.y = SosUtil::maxVal(y1,y2);

	getViewAreaFromMap(maxX+expand,minY-expand,x1,y1,x2,y2);

	pt2.x = SosUtil::maxVal(x1,x2);
	pt2.y = SosUtil::minVal(y1,y2);

	resetIterator(pt1.x,pt1.y,pt2.x,pt2.y);

	return true;
}

//reset the iterator, but only the area of the map in the area defined by the two points
//will be returned by the getNext() method
bool MapViewManager::resetIterator(long x1, long y1, long x2, long y2)
{	
	if(_mapViewerBusy)
		return false;
	
	LOG<<"In resetIterator("<<x1<<","<<y1<<","<<x2<<","<<y2<<")";
	
	SosUtil::ensureSmaller(x1,x2);
	SosUtil::ensureSmaller(y1,y2);

	long dispWidth = getDisplayedMapWidth(), dispHeight = getDisplayedMapHeight();

	if(x2 > dispWidth)
		x2 = dispWidth;

	if(y2 > dispHeight)
		y2 = dispHeight;

	if(x1 < 0)
		x1 = 0;

	if(y1 < 0)
		y1 = 0;

	LOG<<"_getMapFromView = "<<_getMapFromView;

	long temp = 0;
	if(_mapManager->getViewGridMap() && hasMap())
	{
		_finishedDrawingGrid = false;
		_finishedDrawingBackgroundGrid = true;
		_clearedBackground = true;

		if(!_getMapFromView)
		{			
			getMapCoords(x1, y2,_iteratorLeftX,_iteratorBottomY);
			getMapCoords(x2, y1,_iteratorRightX,_iteratorTopY);
		}
		else
		{
			long tempX1 = 0, tempX2 = 0, tempY1= 0, tempY2 = 0;

			//get the bottom left of the box
			getMapCoordsSet(SosUtil::minVal(x1,x2), SosUtil::maxVal(y1,y2),tempX1,tempY1,tempX2,tempY2);
			_iteratorLeftX = SosUtil::minVal(tempX1,tempX2);
			_iteratorBottomY = SosUtil::maxVal(tempY1,tempY2);

			//get the top right of the box
			getMapCoordsSet(SosUtil::maxVal(x1,x2),SosUtil::minVal(y1,y2),tempX1,tempY1,tempX2,tempY2);
			_iteratorRightX = SosUtil::maxVal(tempX1,tempX2);
			_iteratorTopY = SosUtil::maxVal(tempY1,tempY2);			

			LOG<<"Used getMapCoordsSet to get the iterator values";
		}

		
		if(_iteratorLeftX < _mapMinX)
			_iteratorLeftX = _mapMinX;

		if(_iteratorRightX > _mapMaxX)
			_iteratorRightX = _mapMaxX;

		if(_iteratorTopY > _mapMaxY)
			_iteratorTopY = _mapMaxY;

		if(_iteratorBottomY < _mapMinY)
			_iteratorBottomY = _mapMinY;

		long west = 0, north = 0, east = 0, south = 0;
		_mapManager->getGridDimensions(west,north,east,south);

		if(_mapMinX < west
			|| _mapMaxX > east
			|| _mapMaxY > north
			|| _mapMinY < south)		
		{
			_clearedBackground = false;
			_backgroundColour = UNKNOWN_COLOUR;
		}
		else
		{
			_clearedBackground = true;
		}

		if(_iteratorLeftX < west)
			_iteratorLeftX = west;

		if(_iteratorRightX > east)
			_iteratorRightX = east;

		if(_iteratorTopY > north)
			_iteratorTopY = north;

		if(_iteratorBottomY < south)
			_iteratorBottomY = south;
	

		//if the view is completely outside the map, don't draw it
		//just draw the background colour
		if(_iteratorRightX <= _iteratorLeftX || _iteratorBottomY >= _iteratorTopY)
		{
			_finishedDrawingGrid = true;	
		}

		_currentX = _iteratorLeftX;
		_currentY = _iteratorBottomY;

		//if the squaresize is less than 2, then we cannot use a rectangle, since it won't just draw
		//a single pixel.  Therefore, make the _gridDrawObject point to a DrawableScanLine, which
		//draws horizontal lines.
		if(_squareSize > 1)
		{
			_gridDrawObject = &_currentFilledRect;
		}
		else
		{
			_gridDrawObject = &_currentScanLine;
		}
		_gridDrawObject->setViewArea(_displayedArea);
		LOG<<"Set view area of gridDrawObject to width "<<_displayedArea.x<<", height = "<<_displayedArea.y;
	}
	else
	{
		if(!hasMap() && _mapManager->getViewVectorMap())
		{
			_clearedBackground = false;
		}
		_finishedDrawingGrid = true;

		//if we want to view the background grid, then set _finishedDrawingBackgroundGrid to false
		if(_mapManager->getViewVectorMap() && hasMap())
			_finishedDrawingBackgroundGrid = !_viewBackgroundGrid;		

		_listGridBackground.resetIterator();
	}

	if(_mapManager->getViewVectorMap())
	{
		if(_mapManager->getViewGridMap())
		{
			_vectorColour = VECTOR_COLOUR;
		}
		else
		{//
			_clearedBackground = false; //if we're not showing the grid map, then wipe the background
			//before drawing vectors on to it
			_backgroundColour = RGB(255,255,255);
			_vectorColour = RGB(0,0,0);

			LOG<<"Set _backgroundColour to WHITE";
		}
		
		_currentLine.setViewArea(_displayedArea);
		_currentRect.setViewArea(_displayedArea);
		_currentFilledRect.setViewArea(_displayedArea);
		_currentDrawingRobot.setViewArea(_displayedArea);
	
	}
	else if(_viewRobot)
	{
		_currentDrawingRobot.setViewArea(_displayedArea);
	}

	//should we draw the voronoi diagram?
	if(_viewVoronoi && _mapManager->hasVoronoi())
	{		
		_finishedDrawingVoronoi = false;		
	}
	else
	{
		_finishedDrawingVoronoi = true;
	//	_finishedDrawingVoronoiVertices = true; //TO DO - GIVE THIS IT'S OWN IF STMT
	}

	if(_viewVoronoiVertices && _mapManager->hasVoronoi())
	{
		_finishedDrawingVoronoiVertices = false;
	}
	else
	{
		_finishedDrawingVoronoiVertices = true;
	}

	if(_viewDelaunay && _mapManager->hasDelaunay())
	{
		_finishedDrawingDelaunay = false;
	}
	else 
	{
		_finishedDrawingDelaunay = true;
	}

	//should we draw the path(s)?
	if(_viewPath && _mapManager->hasPath())
	{
		_finishedDrawingPath = false;
	}
	else
	{
		_finishedDrawingPath = true;
	}

	//if we have robot runs and want to view them....
	if(_viewRobotRun && _mapManager->hasRobotRun())
	{
		_finishedDrawingRobotRun = false;
		_getNextRobotRunList = true;
		_currentRobotRunColourNum = -1;
	}
	else
	{
		_finishedDrawingRobotRun = true;
	}

	if(_viewBorder)
	{
		_drewBorder = false;
	}
	else
	{
		_viewBorder = true;
	}

	LOG<<"_iteratorLeftX = "<<_iteratorLeftX<<", _iteratorRightX = "<<_iteratorRightX;
	LOG<<"_iteratorTopY = "<<_iteratorTopY<<", _iteratorBottomY = "<<_iteratorBottomY;
	LOG<<"_mapMinX = "<<_mapMinX<<", _mapMaxX = "<<_mapMaxX<<", _mapMinY = "<<_mapMinY<<", _mapMaxY = "<<_mapMaxY;

	return true;
}



bool MapViewManager::repaint(IScreenPainter* painter)
{
	LOG<<"At start of repaint"<<endl;
	bool retval = true;	

	PointXYLong pt1, pt2;
	PointXYLong tempPt1,tempPt2;
	static float value = 0;
	float value2 = 0;
	int valInt = 0, valInt2 = 0;
	long lookaheadX = 0;
	long counter = 0, arrayCounter = 0;
	int i = 0;

	long resolution = _mapManager->getResolution();

	if(!hasMap())
		return false;

	if(!_mapViewerBusy)
	{
		_mapViewerBusy = true;
	}
	else
	{
		return false;
	}

	if(!_clearedBackground)
	{
		_clearedBackground = true;

		pt1.setPoints(0,0);
		
		painter->paintObject(getDrawableRectangleFilled(pt1,_displayedArea,_backgroundColour));		
	}

	long baseX = 0;	
	
	float **tempRow2D = new float*[_viewReduceFactor];
	long arraySize = _iteratorRightX - _iteratorLeftX +1 +_viewReduceFactor;
	for(i = 0; i< _viewReduceFactor; i+=_sampleStep)
	{
		tempRow2D[i] = new float[arraySize];
	}

	float *tempRow = tempRow2D[0];

	ICopyRow2D<float>* mapObject = 0;

	LOG<<"_finishedDrawingGrid = "<<_finishedDrawingGrid;

	if(!_finishedDrawingGrid)
	{
		LOG<<"_iteratorLeftX = "<<_iteratorLeftX<<", _iteratorRightX = "<<_iteratorRightX;
		LOG<<"_iteratorTopY = "<<_iteratorTopY<<", _iteratorBottomY = "<<_iteratorBottomY;

		mapObject = _mapManager->getGridReader();

		//if the map is not too big, then just return the cell value as is
		//otherwise we have to impose a view on the map
		if(!_getMapFromView)
		{	
			retval = mapObject->copyRow(tempRow,_currentY,_iteratorLeftX,_iteratorRightX);

			while(!_finishedDrawingGrid)
			{
				//value =  _gridLayer.read(_currentX,_currentY);//read from the layer object
				value = tempRow[_currentX - _iteratorLeftX];
				
				valInt = int((100 - (value *100)) * 2.55) ;
				
				lookaheadX = _currentX+1;
				
				//value2 =  _gridLayer.read(lookaheadX,_currentY);//read from the layer object
				value2 = tempRow[lookaheadX - _iteratorLeftX];
				valInt2 = int((100 - (value2 *100)) * 2.55) ;
				
			
				while(valInt == valInt2 && lookaheadX <= _iteratorRightX)
				{
					lookaheadX++;
					//value2 =  _gridLayer.read(lookaheadX,_currentY);//read from the layer object
					value2 = tempRow[lookaheadX-_iteratorLeftX];
					valInt2 = int((100 - (value2 *100)) * 2.55) ;
				}
				lookaheadX--;//go back one position, because either the cell it's on is a different value, or
							 //we've passed the right of the map

				//retval = getViewAreaFromMap(_currentX, _currentY, lookaheadX, _currentY,pt1.x, pt1.y,pt2.x, pt2.y);
				getViewAreaFromMapStandardView(_currentX, _currentY, lookaheadX, _currentY,pt1.x, pt1.y,pt2.x, pt2.y);

				_gridDrawObject->setPoints(pt1,pt2);
				if(valInt >= 0 && valInt <= 255)
				{
					//_gridDrawObject->setColour(RGB(valInt,valInt,valInt));
					_gridDrawObject->setColour(_gridColours[valInt]);
				}
				else
				{
					_gridDrawObject->setColour(UNKNOWN_COLOUR);
				}
				_currentX = lookaheadX+1;

				if(_currentX > _iteratorRightX) //if we've passed the right of the map, go down to the next row
				{
					_currentX = _iteratorLeftX;
					_currentY++;
					retval = mapObject->copyRow(tempRow,_currentY,_iteratorLeftX,_iteratorRightX);
				}

				painter->paintObject(_gridDrawObject);
				
				if(_currentY > _iteratorTopY)  //if we've passed the bottom-most row, we're finished drawing the grid
				{
					_finishedDrawingGrid = true;
				}				
			}			
		}
		else
		{	
			LOG<<"repaint() in _getMapFromView"<<endl;

			if(_viewCellMode == VIEW_CELL_AVG)
			{				
				for(i = 0; i< _viewReduceFactor; i+= _sampleStep)//++)
				{
					retval = mapObject->copyRow(tempRow2D[i],_currentY+i,_iteratorLeftX,_iteratorRightX+ _viewReduceFactor);	
				}

				while(!_finishedDrawingGrid)
				{
					value = 0;
					counter = 0;

					baseX = _currentX - _iteratorLeftX;

					for(long x = 0; x < _viewReduceFactor; x += _sampleStep)
					{						
						for(long y = 0; y < _viewReduceFactor; y += _sampleStep)
						{						
							//value +=  _gridLayer.read(_currentX+ x,_currentY+y);//read from the layer object	
							value += tempRow2D[y][baseX + x];
							counter ++;
						}
					}
	
					value /= counter;//(_viewReduceFactor * _viewReduceFactor);
					
					valInt = int( (100 - (value *100)) * 2.55 );
					lookaheadX = _currentX;

					do
					{
						lookaheadX += _viewReduceFactor;
						baseX = lookaheadX - _iteratorLeftX;
					
						counter = 0;

						for(long x = 0; x < _viewReduceFactor && (x + baseX) < arraySize; x += _sampleStep)
						{
							for(long y = 0; y < _viewReduceFactor; y += _sampleStep)
							{
								value2 += tempRow2D[y][x + baseX];//(lookaheadX - _iteratorLeftX)];
								counter ++;
							}
						}

						value2 /= counter;
						valInt2 = int( (100 - (value2 *100)) * 2.55 );

					}while(valInt == valInt2 && lookaheadX <= _iteratorRightX - _viewReduceFactor);
					lookaheadX -= _viewReduceFactor;

					getViewAreaFromMapUsingView(_currentX, _currentY, lookaheadX, _currentY,pt1.x, pt1.y,pt2.x, pt2.y);
					_gridDrawObject->setPoints(pt1,pt2);

					if(value < 0 )
					{
						if(value < -0.5)
							_gridDrawObject->setColour(UNKNOWN_COLOUR);
						else
							_gridDrawObject->setColour(WHITE_COLOUR);
					}
					else if(value <= 1)
					{
						//_gridDrawObject->setColour(RGB(valInt,valInt,valInt));
						_gridDrawObject->setColour(_gridColours[valInt]);
					}
					else
					{
						_gridDrawObject->setColour(UNKNOWN_COLOUR);
						
					}					

					_currentX = lookaheadX + _viewReduceFactor;
					painter->paintObject(_gridDrawObject);

					if(_currentX > _iteratorRightX - _viewReduceFactor)
					{
						_currentX = _iteratorLeftX;
						_currentY+=_viewReduceFactor;
						
						for(i = 0; i< _viewReduceFactor; i+= _sampleStep)//++)
						{
							retval = mapObject->copyRow(tempRow2D[i],_currentY+i,_iteratorLeftX,_iteratorRightX+ _viewReduceFactor);
						}

						if(_currentY > _iteratorTopY)
						{
							_finishedDrawingGrid = true;
						}
					}
					
					
				}
				LOG<<"repaint() at end of VIEW_CELL_AVG"<<endl;
			}
			else if(_viewCellMode == VIEW_CELL_MAX)
			{
				LOG<<"repaint() in VIEW_CELL_MAX"<<endl;
				for(i = 0; i< _viewReduceFactor; i+=_sampleStep)
				{
					retval = mapObject->copyRow(tempRow2D[i],_currentY+i,_iteratorLeftX,_iteratorRightX);
				}
				while(!_finishedDrawingGrid)
				{
					value = -1;
					baseX = _currentX - _iteratorLeftX;
					for(long x = 0; x < _viewReduceFactor; x += _sampleStep)
					{
						for(long y = 0; y < _viewReduceFactor; y += _sampleStep)
						{
							//value = SosUtil::maxVal(value,_gridLayer.read(_currentX+ x,_currentY+y));
							value = SosUtil::maxVal(value,tempRow2D[y][baseX+ x]);//read from the layer object	
						}
					}

					valInt = int( (100 - (value *100)) * 2.55 );
					lookaheadX = _currentX;

					do
					{
						lookaheadX += _viewReduceFactor;		
						baseX = lookaheadX - _iteratorLeftX;
						value2 = -1;
						for(long x = 0; x < _viewReduceFactor && (x + baseX) < arraySize; x += _sampleStep)
						{
							for(long y = 0; y < _viewReduceFactor; y += _sampleStep)
							{
								//value2 =  SosUtil::maxVal(value2,_gridLayer.read(lookaheadX+ x,_currentY+y));
								value2 =  SosUtil::maxVal(value2,tempRow2D[y][baseX+ x]);//read from the layer object	
							}
						}
						
						valInt2 = int( (100 - (value2 *100)) * 2.55 );

					}while(valInt == valInt2 && lookaheadX <= _iteratorRightX - _viewReduceFactor);
					lookaheadX -= _viewReduceFactor;

					getViewAreaFromMapUsingView(_currentX, _currentY, lookaheadX, _currentY,pt1.x, pt1.y,pt2.x, pt2.y);

					_gridDrawObject->setPoints(pt1,pt2);

					if(value < 0 || value > 1)
					{
						_gridDrawObject->setColour(UNKNOWN_COLOUR);
					}
					else
					{
						//_gridDrawObject->setColour(RGB(valInt,valInt,valInt));
						_gridDrawObject->setColour(_gridColours[valInt]);
					}
					

					_currentX = lookaheadX + _viewReduceFactor;

					if(_currentX > _iteratorRightX)
					{
						_currentX = _iteratorLeftX;
						_currentY+=_viewReduceFactor;
						for(i = 0; i< _viewReduceFactor; i+=_sampleStep)
						{
							retval = mapObject->copyRow(tempRow2D[i],_currentY+i,_iteratorLeftX,_iteratorRightX);
						}
					}
					
					if(_currentY > _iteratorTopY)
					{
						_finishedDrawingGrid = true;
					}
					painter->paintObject(_gridDrawObject);
				}
			}
			else if(_viewCellMode == VIEW_CELL_MIN)
			{
				for(i = 0; i< _viewReduceFactor; i+=_sampleStep)
				{
					retval = mapObject->copyRow(tempRow2D[i],_currentY+i,_iteratorLeftX,_iteratorRightX);
				}
				while(!_finishedDrawingGrid)
				{
					value = 1;
					baseX = _currentX - _iteratorLeftX;
					for(long x = 0; x < _viewReduceFactor; x += _sampleStep)
					{
						for(long y = 0; y < _viewReduceFactor; y += _sampleStep)
						{
							value = SosUtil::minVal(value,tempRow2D[y][baseX + x]);							
						}
					}

					valInt = int( (100 - (value *100)) * 2.55 );
					lookaheadX = _currentX;

					do
					{
						lookaheadX += _viewReduceFactor;
						baseX = lookaheadX - _iteratorLeftX;
						value2 = 1;
						for(long x = 0; x < _viewReduceFactor && (x + baseX) < arraySize; x += _sampleStep)
						{
							for(long y = 0; y < _viewReduceFactor; y += _sampleStep)
							{
								value2 = SosUtil::minVal(value2,tempRow2D[y][baseX + x]);;
							}
						}
						valInt2 = int( (100 - (value2 *100)) * 2.55 );

					}while(valInt == valInt2 && lookaheadX <= _iteratorRightX - _viewReduceFactor);
					lookaheadX -= _viewReduceFactor;

					getViewAreaFromMapUsingView(_currentX, _currentY, lookaheadX, _currentY,pt1.x, pt1.y,pt2.x, pt2.y);

					_gridDrawObject->setPoints(pt1,pt2);

					if(value < 0 || value > 1)
					{
						_gridDrawObject->setColour(UNKNOWN_COLOUR);
					}
					else
					{
						//_gridDrawObject->setColour(RGB(valInt,valInt,valInt));
						_gridDrawObject->setColour(_gridColours[valInt]);
					}
					

					_currentX = lookaheadX + _viewReduceFactor;

					if(_currentX > _iteratorRightX)
					{
						_currentX  = _iteratorLeftX;
						_currentY += _viewReduceFactor;
						for(i = 0; i< _viewReduceFactor; i+=_sampleStep)
						{
							retval = mapObject->copyRow(tempRow2D[i],_currentY+i,_iteratorLeftX,_iteratorRightX);
						}
					}
					
					if(_currentY > _iteratorTopY)
					{
						_finishedDrawingGrid = true;
					}
					painter->paintObject(_gridDrawObject);
				}
				LOG<<"repaint() at end of VIEW_CELL_MIN"<<endl;
			}
		}
	}	

	for(i = 0; i< _viewReduceFactor; i+=_sampleStep)
	{
		delete[] tempRow2D[i];
		tempRow2D[i] = 0;
	}
	delete [] tempRow2D;
	tempRow2D = 0;

	LOG<<"repaint() finished drawing grid"<<endl;
	LineXYLayer line;
	long x1 = 0, y1 = 0, x2 = 0, y2 = 0;
	DrawableObject* obj = 0;
	int type = 0;
	
	PointXYLong displayedArea(getDisplayedMapWidth(),getDisplayedMapHeight());

	//if the grid map isn't visible, draw a light grey background grid
	if(!_mapManager->getViewGridMap() && !_finishedDrawingBackgroundGrid )
	{
		LOG<<"Drawing the background grid";
		//read the first object - if it is a filled rectangle, then the squaresize is so small
		//that we're just drawing a filled rectangle, not the grid line-by-line
		if(_listGridBackground.readHead(line))
		{
			if(line.type == OBJECT_TYPE_RECTANGLE)
			{
				obj = createDrawableObject(line);
				if(obj != 0)
				{
					obj->setColour(BACKGROUND_GRID_COLOUR);
					_finishedDrawingBackgroundGrid = true;
					painter->paintObject(obj);
				}
			}
		}

		while(_listGridBackground.readNext(line))
		{
			obj = createDrawableObject(line);
			if(obj != 0)
			{
			//	obj->getPoints(pt1,pt2);

				obj->setColour(BACKGROUND_GRID_COLOUR);
				painter->paintObject(obj);
			}
		}
		_finishedDrawingBackgroundGrid = true;

	}

	IListReader<LineXYLayer>* listObjects = _mapManager->getAllObjectsReader();
	listObjects->resetIterator();

	LOG<<"_mapManager->getViewVectorMap() = "<<_mapManager->getViewVectorMap();

	ListUnordered<LineXYLayer> robotList;
	if(_viewRobot)
	{
		if(!_mapManager->getViewVectorMap())
		{
			//if the vector map is not visible, then just search for robots and draw them
			while(listObjects->readNext(line))
			{
				if(line.type == OBJECT_TYPE_ROBOT)
				{
					obj = createDrawableObject(line);
					if(obj == 0)
					{
						//LOG<<"Failed to create a drawable robot";
						continue;
					}
					obj->setColour(_robotColour);
					painter->paintObject(obj);	
				}
			}
		}
		else
		{
			//skip all robots, and paint them last, so they are visible over the normal vectors
			while(listObjects->readNext(line))
			{
				if(line.type == OBJECT_TYPE_ROBOT)
				{
					robotList.push(line);
					continue;
				}
				obj = createDrawableObject(line);
				if(obj == 0)
				{
					//LOG<<"Failed to create a drawable object";
					continue;
				}				

				obj->setColour(_vectorColour);
				painter->paintObject(obj);			
			}

			if(robotList.getListSize() > 0)
			{
				while(robotList.popHead(line))
				{
					obj = createDrawableObject(line);
					if(obj == 0)
					{
					//	LOG<<"Failed to create a drawable robot";
						continue;
					}				

					obj->setColour(_robotColour);
					painter->paintObject(obj);
				}
			}
		}
	}
	else if(_mapManager->getViewVectorMap())
	{		
		LOG<<"Going to draw all the "<<listObjects->getListSize()<<" vectors now";
		while(listObjects->readNext(line))
		{
			LOG<<"Repaint: Got object "<<line;
			obj = createDrawableObject(line);
			if(obj == 0)
			{
				//LOG<<"Failed to create a drawable object";
				continue;
			}
			obj->setColour(_vectorColour);
			painter->paintObject(obj);			
		}
	} 

	IListReader<LineXY>* listVoronoiLines = _mapManager->getVoronoiLinesReader();
	listVoronoiLines->resetIterator();

	LOG<<"_finishedDrawingVoronoi = "<<_finishedDrawingVoronoi;

	LineXY lineFloat;
	if(!_finishedDrawingVoronoi)
	{
		line.type = OBJECT_TYPE_LINE;
		
		while(listVoronoiLines->readNext(lineFloat))
		{
			//LOG<<"Painting voronoi line "<<lineFloat;
			line = lineFloat;
			obj = createDrawableObject(line);
			if(obj == 0)
			{
				continue;
			}
			obj->setColour(_voronoiColour);
			painter->paintObject(obj);	
		}

		_finishedDrawingVoronoi = true;
	}


	LOG<<"_finishedDrawingDelaunay = "<<_finishedDrawingDelaunay;

	if(!_finishedDrawingDelaunay)
	{		
		IListReader<LineXY>* listDelaunayLines = _mapManager->getDelaunayLinesReader();
		listDelaunayLines->resetIterator();

		line.type = OBJECT_TYPE_LINE;
		
		while(listDelaunayLines->readNext(lineFloat))
		{
			//LOG<<"Painting delaunay line "<<lineFloat;
			line = lineFloat;
			obj = createDrawableObject(line);
			if(obj == 0)
			{
				continue;
			}
			obj->setColour(_delaunayColour);
			painter->paintObject(obj);	
		}

		_finishedDrawingDelaunay = true;
	}
	
	PointXY ptFloat;

	IListReader<PointXY>* listVoronoiVertices = _mapManager->getVoronoiVerticesReader();	

	if(!_finishedDrawingVoronoiVertices)
	{
		listVoronoiVertices->resetIterator();

		line.type = OBJECT_TYPE_RECTANGLE_FILLED;
		while(listVoronoiVertices->readNext(ptFloat))
		{			
			line.pt1.x = ptFloat.x;
			line.pt1.y = ptFloat.y;
			line.pt2.x = ptFloat.x;
			line.pt2.y = ptFloat.y;

			obj = createDrawableObject(line);
			if(obj == 0)
			{
				continue;
			}
			obj->setColour(_voronoiColour);
			
			//since a square is shown centred on a cell, and for the voronoi diagram it must be
			//centred on a voronoi junction, shift it by it's floating point difference to the float
			obj->getPoints(tempPt1,tempPt2);

			SosUtil::ensureSmaller(tempPt1.x,tempPt2.x);
			SosUtil::ensureSmaller(tempPt1.y,tempPt2.y);
			tempPt1.x -= 2;
			tempPt2.x += 2;
			tempPt1.y -= 2;
			tempPt2.y += 2;	

			obj->setPoints(tempPt1,tempPt2);
			painter->paintObject(obj);
		}
		_finishedDrawingVoronoiVertices = true;
	}

	LineXYLong lineLong;

	IListReader<LineXYLong>* listPathLines = _mapManager->getPathLinesReader();

	LOG<<"_finishedDrawingPath = "<<_finishedDrawingPath;
	LOG<<"number of path lines = "<<listPathLines->getListSize();

	if(!_finishedDrawingPath)
	{
		listPathLines->resetIterator();

		while(listPathLines->readNext(lineLong))
		{
			_mapManager->gridToMm(lineLong.pt1.x,lineLong.pt1.y,lineLong.pt1.x,lineLong.pt1.y);
			_mapManager->gridToMm(lineLong.pt2.x,lineLong.pt2.y,lineLong.pt2.x,lineLong.pt2.y);
			line = LineXYLongToFloat(lineLong,false);
			line.layer = 0;
			line.type = OBJECT_TYPE_LINE;
			LOG<<"Drawing Path line from ("<<line.pt1.x<<","<<line.pt1.y<<") to ("<<line.pt2.x<<","<<line.pt2.y<<")";

			obj = createDrawableObject(line);
			if(obj == 0)
			{
				LOG<<"Skipping the path line "<<line<<" because couldn't create a drawable object from it";
				continue;
			}
			obj->setColour(_pathColour);
			painter->paintObject(obj);
		}
		_finishedDrawingPath = true;
	}

	SosPose pose(0,0,0);
	ListUnordered<SosPose> * list = 0;


	
	while(!_finishedDrawingRobotRun)
	{
		LOG<<"repaint() in !_finishedDrawingRobotRun"<<endl;
		if(_getNextRobotRunList )
		{
			_getNextRobotRunList = false;

			if(_listRobotRunLists.readNext(_currentRobotRunList))
			{
				_currentRobotRunList->resetIterator();
				_currentRobotRunColourNum++;
				_currentRobotRunColour = _robotRunColours->get(_currentRobotRunColourNum);
			}
			else
			{
				_finishedDrawingRobotRun = true;
			}			
		}
		line.layer = 0;
		line.type = OBJECT_TYPE_RECTANGLE_FILLED;
		while(!_finishedDrawingRobotRun && _currentRobotRunList->readNext(pose))
		{
			if(SosUtil::between(long((pose.getX())/resolution),_mapMinX,_mapMaxX) &&
				SosUtil::between(long((pose.getY())/resolution),_mapMinY,_mapMaxY))
			{
				line.pt1.x = float(pose.getX()/(float)resolution);
				line.pt1.y = float(pose.getY()/(float)resolution);
				line.pt2 = line.pt1;
				obj = createDrawableObject(line);
				if(obj == 0)
				{
					continue;
				}
				obj->setColour(_currentRobotRunColour);
				obj->getPoints(pt1,pt2);

				pt1.x += 1;
				pt2.x -= 1;
				pt1.y += 1;
				pt2.y -= 1;
				obj->setPoints(pt1,pt2);
				painter->paintObject(obj);
			}
		}

		if(!_finishedDrawingRobotRun)
		{
			_getNextRobotRunList = true;			
		}
	}

	PointXYLong viewArea = _displayedArea;
	viewArea.x += 2;
	viewArea.y += 2;


	if(!_drewBorder)
	{
		LOG<<"repaint() in !_drewBorder"<<endl;
		_drewBorder = true;
		_currentRect.setPoints(0,0,getDisplayedMapWidth(), getDisplayedMapHeight());
		_currentRect.setColour(RGB(0,0,0));
		_currentRect.setViewArea(viewArea);

		painter->paintObject(&_currentRect);
	}

	if(!_finishedDrawingRuler)
	{
		LOG<<"repaint() in !_finishedDrawingRuler"<<endl;
		_finishedDrawingRuler = true;
		painter->paintObject(_rulerObj);
	}

	_changedMaxX = _changedMinX = _changedMinY = _changedMaxY = 0;
	_changedValuesUnset = true;

	_pathChanged=_gridChanged=_vectorsChanged=_voronoiChanged=false;
	LOG<<"At end of repaint()"<<endl;

	_mapViewerBusy = false;
	return true;
}


long MapViewManager::getViewArea(int direction)
{
	switch(direction)
	{
	case NORTH:
		return _mapMaxY;
	case EAST:
		return _mapMaxX;
	case SOUTH:
		return _mapMinY;
	case WEST:
		return _mapMinX;
	}

	return 0;
}

long MapViewManager::getDisplayedMapWidth()
{
//	LOGENTRY("getDisplayedMapWidth")
	return _displayedArea.x;
//	LOGEXIT("getDisplayedMapWidth")
}

PointXYLong MapViewManager::getViewSize()
{
	PointXYLong pt(0,0);
	long x1, y1,x2,y2;
	x1 = x2 = y1 = y2 = 0;

	if(_squareSize == 0)
		return pt;
	
	getViewAreaFromMap(_mapMaxX,_mapMinY,x1,y1,x2,y2);		
	pt.y = SosUtil::maxVal(y1,y2);
	pt.x = SosUtil::maxVal(x1,x2);		
	
	return pt;
}

long MapViewManager::getDisplayedMapHeight()
{
	return _displayedArea.y;
}

void MapViewManager::setPoint(long x, long y, float value)
{
	if(!hasMap())
		return ;

	LineXYLong currentMapSize ;
	_mapManager->getDimensions(currentMapSize.pt1.x,currentMapSize.pt1.y,currentMapSize.pt2.x,currentMapSize.pt2.y);
		
	long mapX = 0, mapY = 0;

	float xFloat = 0, yFloat = 0;
	screenToGrid(x,y,xFloat,yFloat);
	mapX = (long)xFloat;
	mapY = (long)yFloat;

	long minX=mapX - _brushSize/2, minY=mapY-_brushSize/2,maxX=mapX+_brushSize/2,maxY=mapY+_brushSize/2;
	float val = 0;

	for(x = minX; x<=maxX; x++)
	{
		for( y = minY; y <= maxY; y++)
		{
			if(_mapManager->getPointVal(x,y) != value)
				_mapManager->setPoint(x,y,value);
		}
	}

	_mapManager->gridToMm(minX,maxY+1,minX,maxY);
	_mapManager->gridToMm(maxX+1,minY,maxX,minY);

	addChangedArea(minX,maxY,maxX,minY,currentMapSize);
}

void MapViewManager::setLine(long x1, long y1, long x2, long y2, float value)
{
	LOGENTRY("setLine")
	if(!hasMap())
		return;

	LineXYLong currentMapSize ;
	_mapManager->getDimensions(currentMapSize.pt1.x,currentMapSize.pt1.y,currentMapSize.pt2.x,currentMapSize.pt2.y);

	LOG<<"setLine got screen coords ("<<x1<<","<<y1<<") -> ("<<x2<<","<<y2<<")";

	long mmX1 = 0, mmY1 = 0, mmX2 = 0, mmY2 = 0;
	screenToMm(x1,y1,mmX1,mmY1);
	screenToMm(x2,y2,mmX2,mmY2);

	LOG<<"setLine setting mm values ("<<mmX1<<","<<mmY1<<") -> ("<<mmX2<<","<<mmY2<<")";

	_mapManager->setLine(mmX1,mmY1,mmX2,mmY2,value);

	setScreenAreaToChanged(x1,y1,x2,y2,2);
//	addChangedArea(mmX1,mmY1,mmX2,mmY2,currentMapSize);
	
	LOGEXIT("setLine")
}


bool MapViewManager::translateToScreenCoords(LineXYLayer& object)
{
	double slope = 0; //this is the 'm' in the line formula y = mx+c
    double yIntercept = 0; //this is the 'c' in the line formula y = mx+c
	bool noSlope = false;

	bool ret1 = false, ret2 = false;

	PointXYLong pt1(0,0),pt2(0,0);
	double x1 = object.pt1.x + 0.5, y1 = object.pt1.y + 0.5,x2 = object.pt2.x + 0.5, y2 = object.pt2.y + 0.5;

	long x1L = 0, x2L = 0, y1L = 0, y2L = 0;

	long mmMinX = 0, mmMaxX = 0, mmMinY = 0, mmMaxY = 0;
	_mapManager->gridToMm(_mapMinX,_mapMinY,mmMinX,mmMinY);
	_mapManager->gridToMm(_mapMaxX,_mapMaxY,mmMaxX,mmMaxY);
	

	//if the object is completely off the screen, return zero
	if((object.pt1.x < mmMinX && object.pt2.x < mmMinX) ||
		(object.pt1.x > mmMaxX && object.pt2.x > mmMaxX) ||
		(object.pt1.y < mmMinY && object.pt2.y < mmMinY) ||
		(object.pt1.y > mmMaxY && object.pt2.y > mmMaxY))
	{
//		LOG<<"The object is not in the area ("<<_mapMinX<<","<<_mapMinY<<") -> ("<<_mapMaxX<<","<<_mapMaxY<<")";
		return false;
	}

	switch(object.type)
	{
	case OBJECT_TYPE_UNDEFINED://there are no selectable objects for these two cases
	case OBJECT_TYPE_POINT:
		return 0;
	case OBJECT_TYPE_LINE:
		mmToScreen(object.pt1.x,object.pt1.y,pt1.x,pt1.y);
		mmToScreen(object.pt2.x,object.pt2.y,pt2.x,pt2.y);
						
		object.pt1 = pt1;
		object.pt2 = pt2;
		return true;

	case OBJECT_TYPE_RECTANGLE:
	case OBJECT_TYPE_RECTANGLE_FILLED:
	case OBJECT_TYPE_ROBOT:
		mmToScreen(object.pt1.x,object.pt1.y,pt1.x,pt1.y);
		mmToScreen(object.pt2.x,object.pt2.y,pt2.x,pt2.y);

		object.pt1 = pt1;
		object.pt2 = pt2;

		return true;
	
	}

	return false;
}

DrawableObject*	MapViewManager::createDrawableObject(LineXYLayer& object)
{
	LOGENTRY("createDrawableObject")

	if(!translateToScreenCoords(object))
	{
		LOG<<"Could not translate object to screen:"<<object;
		return 0;
	}

	switch(object.type)
	{
	case OBJECT_TYPE_UNDEFINED://there are no selectable objects for these two cases
	case OBJECT_TYPE_POINT:
		return 0;
	case OBJECT_TYPE_LINE:
		
		_currentLine.setPoints(object.pt1,object.pt2);
		_currentLine.setViewArea(_displayedArea);
		_currentFilledRect.setColour(RGB(_vectorColour,0,0));

		LOGEXIT("createDrawableObject")
		return &_currentLine;	
	case OBJECT_TYPE_RECTANGLE:
		_currentRect.setPoints(object.pt1,object.pt2);	
		_currentRect.setViewArea(_displayedArea);
		_currentFilledRect.setColour(RGB(_vectorColour,0,0));

	//	LOGEXIT("createDrawableObject")
		return &_currentRect;
	case OBJECT_TYPE_RECTANGLE_FILLED:
		_currentFilledRect.setPoints(object.pt1,object.pt2);
		_currentFilledRect.setViewArea(_displayedArea);
		_currentFilledRect.setColour(RGB(_vectorColour,0,0));		
	//	LOGEXIT("createDrawableObject")
		return &_currentFilledRect;
	case OBJECT_TYPE_ROBOT:
		LOG<<"Creating a drawable ROBOT with points "<<object.pt1<<" and "<<object.pt2;
		_currentDrawingRobot.setPoints(object.pt1,object.pt2);
		_currentDrawingRobot.setViewArea(_displayedArea);
		_currentDrawingRobot.setColour(RGB(_vectorColour,120,10));	
		_currentDrawingRobot.setOrientation(object.value);
		return &_currentDrawingRobot;
		break;
	
	}

	LOGEXIT("createDrawableObject")
	return 0;
}

//not sure this is still needed.....
LineXYLayer	MapViewManager::fixRobotSize(LineXYLayer object)
{
	if(object.type != OBJECT_TYPE_ROBOT)
	{
		return object;
	}

	PointXY centrePt(0,0);

	object.pt1.x = SosUtil::midWay(object.pt1.x,object.pt2.x);
	object.pt1.y = SosUtil::midWay(object.pt1.y,object.pt2.y);

	double robotWidth = ((double)ROBOT_RADIUS / (double)_mapManager->getResolution());
	
	object.pt2 = object.pt1;

	object.pt1.x -= robotWidth;
	object.pt1.y += robotWidth;
	object.pt2.x += robotWidth;
	object.pt2.y -= robotWidth;

	return object;
}


float MapViewManager::getPointVal(long x, long y)
{
	if(!hasMap())
	{
		return -1;
	}
	
	float gridX = 0,gridY = 0;
	screenToGrid(x,y,gridX,gridY);

	return _mapManager->getPointVal(gridX,gridY);
}

//fills in a rectangle with the specified value based on two coordinates
//in the window view
void MapViewManager::setRectangle(long x1, long y1, long x2, long y2, float value)
{
	LOGENTRY("setRectangle")
	if(!hasMap())
		return;

	long mmX1 = 0, mmY1 = 0, mmX2 = 0, mmY2 = 0;
	screenToMm(x1,y1,mmX1,mmY1);
	screenToMm(x2,y2,mmX2,mmY2);

	LineXYLong currentMapSize ;
	_mapManager->getDimensions(currentMapSize.pt1.x,currentMapSize.pt1.y,currentMapSize.pt2.x,currentMapSize.pt2.y);


	_mapManager->setRectangle(mmX1,mmY1,mmX2,mmY2,value);	

	addChangedArea(mmX1,mmY1,mmX2,mmY2,currentMapSize);

	LOGEXIT("setRectangle")
}

//fills in a rectangle with the specified value based on two coordinates
//in the window view
void MapViewManager::setRectangleFilled(long x1, long y1, long x2, long y2, float value)
{
	LOGENTRY("setRectangleFilled")
	if(!hasMap())
		return;

	long mmX1 = 0, mmY1 = 0, mmX2 = 0, mmY2 = 0;
	screenToMm(x1,y1,mmX1,mmY1);
	screenToMm(x2,y2,mmX2,mmY2);

	LineXYLong currentMapSize ;
	_mapManager->getDimensions(currentMapSize.pt1.x,currentMapSize.pt1.y,currentMapSize.pt2.x,currentMapSize.pt2.y);

	_mapManager->setRectangleFilled(mmX1,mmY1,mmX2,mmY2,value);	

	addChangedArea(mmX1,mmY1,mmX2,mmY2,currentMapSize);
	
	LOGEXIT("setRectangleFilled")
}


void MapViewManager::setViewPath(bool viewPath)
{
	_viewPath = viewPath;
	_finishedDrawingRuler = false;	
}

void MapViewManager::setViewGridMap(bool viewGridMap)
{
	_mapManager->setViewGridMap(viewGridMap);
	_finishedDrawingRuler = false;
}


void MapViewManager::fillArea(long xVal, long yVal,float valueToSet,double tolerance)
{
	LOGENTRY("fillArea")
	LineXYLong retval;
	if(!hasMap())
	{
		return ;
	}

	screenToMm(xVal,yVal,xVal,yVal);

	_mapManager->fillArea(xVal,yVal,valueToSet,tolerance);

	setAllScreenToChanged();//this can be refined

	LOGEXIT("fillArea")
}

bool MapViewManager::convertGridToLineWithVoronoi(float minThreshold, float maxThreshold,
												  bool filterByCellValue, float valueToSet)
{
	if(!hasMap())
		return false;

	_mapManager->convertGridToLineWithVoronoi(minThreshold,maxThreshold,filterByCellValue,valueToSet);
	
	setAllScreenToChanged();
	_parentWindow->updateToolbarButtons();	

	return true;
}


bool MapViewManager::convertGridToLine(float threshold)
{
	if(!hasMap())
		return false;

	_mapManager->convertGridToLine(threshold);

	_mapManager->setViewVectorMap(true);
	_mapManager->setViewGridMap(false);
	setAllScreenToChanged();
	_parentWindow->updateToolbarButtons();
	
	return true;
}

LineXY	MapViewManager::LineXYLongToFloat(LineXYLong l, bool isRectangle = false)
{
	bool switchedX = SosUtil::ensureSmaller(l.pt1.x,l.pt2.x);
	bool switchedY = SosUtil::ensureSmaller(l.pt1.y,l.pt2.y);

	float xMinDiff = 0.5f,yMinDiff = 0.5f, xMaxDiff = 0.5f,yMaxDiff = 0.5f;
	if(isRectangle)
	{
		xMinDiff = -0.5f;
		yMinDiff = -0.5f; 
		xMaxDiff = 1.0f;
		yMaxDiff = 1.0f;
	}
	LineXY lf;
	if(switchedX)
	{
		lf.pt1.x = l.pt2.x +xMaxDiff;
		lf.pt2.x = l.pt1.x +xMinDiff;
	}
	else
	{
		lf.pt1.x = l.pt1.x +xMinDiff;
		lf.pt2.x = l.pt2.x +xMaxDiff;
	}
	if(switchedY)
	{
		lf.pt1.y = l.pt2.y+yMaxDiff;
		lf.pt2.y = l.pt1.y+yMinDiff;
	}
	else
	{
		lf.pt1.y = l.pt1.y+yMinDiff;
		lf.pt2.y = l.pt2.y+yMaxDiff;
	}
	return lf;
}



bool MapViewManager::averageGridMap(char* filePath)
{
	bool retval = _mapManager->averageGridMap(filePath);
	
	if(retval)
	{		
		resetView();		
		setAllScreenToChanged();
	}

	return retval;
}


double MapViewManager::correlateMap(char* filePath, bool popupAnswer)
{
	double retval = _mapManager->correlateMap(filePath);

	if(!popupAnswer)
		return retval;

	char buffer[256] = {0};
	_parentWindow->popupOk("Correlation Value", SosUtil::doubleToString(retval,4,buffer));
	return retval;
}

double MapViewManager::mapScoreMap(char* filePath,bool justCompareOccAreas,  bool popupAnswer)
{
	double retval = _mapManager->mapScoreMap(filePath,justCompareOccAreas);

	if(!popupAnswer)
		return retval;

	char buffer[256] = {0};
	_parentWindow->popupOk("Map Score Value", SosUtil::doubleToString(retval,4,buffer));
	return retval;
}



DrawableObject* MapViewManager::getDrawableRectangleFilled(PointXYLong pt1, PointXYLong pt2, 
															   DWORD colour )
{
	_currentFilledRect.setPoints(pt1,pt2);
	_currentFilledRect.setViewArea(_displayedArea);
	_currentFilledRect.setColour(colour);
	
	return &_currentFilledRect;
}


void MapViewManager::setResolution(long res)
{
	//if the new resolution is the same as the old one, then do nothing
	if(res == _mapManager->getResolution())
	{
		return;
	}

	if(_mapManager->hasPath() || _mapManager->hasVoronoi())
	{
		if(_parentWindow->popupOkCancel(STD_CONFIRM_TITLE,"This will discard all Paths and Voronoi diagrams. Are you sure?"))
		{
			return;
		}
	}

	_mapManager->setResolution(res);
	setAllScreenToChanged();
	calcSquareSize();
}

void MapViewManager::clearVoronoi()
{
	_mapManager->clearVoronoi();
	setAllScreenToChanged();
}

void MapViewManager::clearDelaunay()
{
	_mapManager->clearDelaunay();
	setAllScreenToChanged();
}

void MapViewManager::clearPaths()
{
	_mapManager->clearPaths();
	setAllScreenToChanged();
}

void MapViewManager::clearRobotRun()
{
	_mapManager->clearRobotRun();
	setAllScreenToChanged();
}

bool MapViewManager::clearRobots()
{
	bool retval = _mapManager->clearRobots();
	if(retval)
		setAllScreenToChanged();

	return retval;
	
}

void MapViewManager::clearVectors()
{
	_mapManager->clearVectors();
	setAllScreenToChanged();
}

void MapViewManager::clearGridMap()
{
	_mapManager->clearGridMap();
	setAllScreenToChanged();

}

void MapViewManager::matchScreenToMap()
{
	long viewHeightUsed = 0, _viewWidthUsed = 0;

	long north = 0, south = 0, west = 0, east = 0;
	_mapManager->getGridDimensions(west,north,east,south);

	viewHeightUsed = north - south + 1;
	_viewWidthUsed = east - west + 1;

	_oldviewWidth = _viewWidth;
	_oldviewHeight = _viewHeight;

	_rulerWasVisible = _rulerObj->isVisible();

	_rulerObj->setVisible(false);

	resetView();

	resizeWindow(_viewWidthUsed,viewHeightUsed);
}

void MapViewManager::unmatchScreenToMap()
{
	long viewHeightUsed = 0, _viewWidthUsed = 0;

	//both of these should be -1 if matchScreenToMap wasn't called - so do nothing if they are -1
	if(_oldviewWidth == -1 || _oldviewHeight == -1)
		return;

	_rulerObj->setVisible(_rulerWasVisible);

	resizeWindow(_oldviewWidth,_oldviewHeight);
	resetView();

	_oldviewWidth = -1;
	_oldviewHeight = -1;
}

void MapViewManager::screenDistToMmDist(long screenX,long screenY, long& mmX, long& mmY)
{
	if(!_getMapFromView )
	{		
		if(_squareSize != 0)
		{
			mmX = (long)((float)screenX / (float)_squareSize) *_mapManager->getResolution();
			mmY = (long)((float)screenY / (float)_squareSize)*_mapManager->getResolution();

			return;
		}
		return;
	}
	else
	{		
		if(_squareSize != 0)
		{
			mmX = (long)(((float)screenX / (float)_squareSize)* _viewReduceFactor)*_mapManager->getResolution();
			mmY = (long)(( (float)screenY / (float)_squareSize)* _viewReduceFactor)*_mapManager->getResolution();	
			
			return ;
		}		
	}
	return ;

}

bool MapViewManager::screenToGrid(long screenX,long screenY, float& gridX, float& gridY)
{
	bool pointInMap = pointInViewArea(screenX,screenY);

	//if the map is not too big to fit easily in the window, then calculate the map coordinates
	//the simple way
	if(!_getMapFromView )
	{		
		if(_squareSize != 0)
		{
			gridX = ( (float)screenX / (float)_squareSize) + float(_mapMinX);
			gridY = (float)_mapMaxY - ((float)screenY / (float)_squareSize)+1;

//	LOG<<"gridY = "<<(float)_mapMaxY<<" - ("<<screenY<<"/"<<_squareSize<<") + 1 = "<<gridY;			
//			LOG<<"Screen ("<<screenX<<","<<screenY<<") = Grid ("<<gridX<<","<<gridY<<")";
		
			//if the requested point is outside the map, return false.  The values in the variables are still
			//changed however
			return pointInMap;
		}
		return false;
	}
	else
	{
		
		if(_squareSize != 0)
		{
			gridX = (((float)screenX / (float)_squareSize)* _viewReduceFactor) + float(_mapMinX);
			gridY = (float)_mapMaxY + 1 - ((( (float)screenY / (float)_squareSize))* _viewReduceFactor);	

			//if the requested point is outside the map, return false.  The values in the variables are still
			//changed however
			
			return pointInMap;
		}		
	}
	return false;
}
bool MapViewManager::gridToScreen(float gridX, float gridY, long& screenX, long& screenY)
{
	bool pointInScreen = true;
	if(!SosUtil::between((double)gridX,(double)_mapMinX,(double)_mapMaxX) || 
		!SosUtil::between((double)gridY,(double)_mapMinY,(double)_mapMaxY))
	{
		pointInScreen = false;
	}

	if(!_getMapFromView)
	{	
		gridX -= (float)_mapMinX;
		//gridY = (float)_mapMaxY - gridY +1;

		screenX = long(gridX * (float)_squareSize);
		screenY = long(((float)_mapMaxY - gridY +1) * (float)_squareSize);
		
//		LOG<<"screenY = ("<<(float)_mapMaxY<<" - "<<gridY<<" + 1) * "<<_squareSize<<" = "<<screenY;
		
//		LOG<<"Grid ("<<gridX<<","<<gridY<<") = Screen ("<<screenX<<","<<screenY<<")";

	}
	else
	{		
		gridX -= (float)_mapMinX;
		gridY = (float)_mapMaxY - gridY +1;
		
		gridX /= (float)_viewReduceFactor;
		gridY /= (float)_viewReduceFactor;
		
		screenX = long(gridX * (float)_squareSize);
		screenY = long(gridY * (float)_squareSize);	
	}

	return pointInScreen;
}

void MapViewManager::mmToScreen(long mmX, long mmY,long& screenX, long& screenY)
{
	float gridX = 0, gridY = 0;
	_mapManager->mmToGrid(mmX,mmY,gridX,gridY);
	gridToScreen(gridX,gridY,screenX,screenY);
}

void MapViewManager::screenToMm(long screenX, long screenY,long& mmX, long& mmY)
{
	float gridX = 0, gridY = 0;
	screenToGrid(screenX,screenY,gridX,gridY);

	_mapManager->gridToMm(gridX,gridY,mmX,mmY);
//	LOG<<"screenToMm, screen ("<<screenX<<","<<screenY<<"), grid ("<<gridX<<","<<gridY<<"), mm ("<<mmX<<","<<mmY<<")";
}

void MapViewManager::screenToMm(long screenX, long screenY,float& mmX, float& mmY)
{
	float gridX = 0, gridY = 0;
	long mmX2 = 0, mmY2 = 0;
	screenToGrid(screenX,screenY,gridX,gridY);
	_mapManager->gridToMm(gridX,gridY,mmX2,mmY2);
	mmX = mmX2;
	mmY = mmY2;
}

bool MapViewManager::generateCSpaceSimple(float min, float max, long distance)
{
	bool retval = _mapManager->generateCSpaceSimple(min,max,distance);

	if(retval)
	{
		resetView();
		setAllScreenToChanged();
	}
	return retval;
}


bool MapViewManager::thresholdMap(float min, float max)
{
	bool retval = _mapManager->thresholdMap(min,max);

	if(retval)
	{
		resetView();
		setAllScreenToChanged();
	}
	return retval;
}

bool MapViewManager::smoothMap(float min, float max)
{
	bool retval = _mapManager->smoothMap(min,max);

	if(retval)
	{
		resetView();
		setAllScreenToChanged();
	}
	return retval;
}


bool MapViewManager::negativeMap()
{
	bool retval = _mapManager->negativeMap();

	if(retval)
	{
		resetView();
		setAllScreenToChanged();
	}
	return retval;
}

//translate the map in millimetres in X and Y directions
bool MapViewManager::translateMap(long xDist, long yDist)
{
	bool retval = _mapManager->translateMap(xDist,yDist);

	if(retval)
	{
		resetView();
		setAllScreenToChanged();
	}
	return retval;
}


//these coordinates are in screen format.  Have to convert them to mm first
bool MapViewManager::cropMap(long x1,long y1, long x2, long y2)
{
	if(!hasMap())
		return false;

	long mmX1=0,mmY1=0,mmX2=0,mmY2=0;
	screenToMm(x1,y1,mmX1,mmY1);
	screenToMm(x2,y2,mmX2,mmY2);
	
	bool retval = _mapManager->cropMap(mmX1,mmY1,mmX2,mmY2);

	if(retval)
	{
		resetView();
		setAllScreenToChanged();
	}
	return retval;
}

