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

#ifndef MAPVIEWMANAGER_H
#define MAPVIEWMANAGER_H

#include "SosUtil.h"
#include "../drawableobjects/DrawableObject.h"
#include "../grid/IncludeAll.h"
#include "fstream.h"
#include "../mapmanager/MapManager.h"

#include "../fileparsers/IncludeAll.h"
#include "WindowCallbackInterface.h"
#include "../mapmanager/WindowMessaging.h"

#include "../list/SosList.h"
#include "VisRuler.h"
#include "../logger/Logger.h"

#define MIN_SQUARE_SIZE					1
#define MAX_VIEW_REDUCTION				1000

//MAX_MAP_REDUCTION_SIZE defines how far the application will allow the map to 
//be zoomed out before it starts to skip cells.  This is a performance enhancement,
//especially for very large maps (100mb+)
#define MAX_MAP_REDUCTION_SIZE			5 

//This defines how many times larger than the screen the map must be before the
//MAX_MAP_REDUCTION_SIZE will be multiplied by a number > 1.  When the map is 50 as large as the screen,
//MAX_MAP_REDUCTION_SIZE is multiplied by 1, when it's 200 times as big as the screen, MAX_MAP_REDUCTION_SIZE is
//multiplied by 4 etc. This enables the display algorithm to scale better to very large maps
#define MAX_MAP_REDUCTION_SIZE_MULFACTOR 10

const DWORD UNKNOWN_COLOUR			= RGB(10,10,200);
const DWORD BACKGROUND_GRID_COLOUR	= RGB(220,220,220);
const DWORD VECTOR_COLOUR			= RGB(200,100,100);
const DWORD WHITE_COLOUR			= RGB(255,255,255);

#define MAX_NUM_ROBOT_RUNS				24

#define MAX_DIST_TO_JOIN_VECTOR			0.2

class MapViewManager : public MapViewManagerCallbackInterface
{
public:

	MapViewManager(WindowCallbackInterface* parentWindow = 0);
	MapViewManager(GridMap<float> *m,WindowCallbackInterface* parentWindow = 0);
	virtual ~MapViewManager();

	bool newMap(long minX, long maxX, long minY, long maxY);
	int addMap(GridMap<float> *m, bool performShallowCopy = false);

	bool loadGridMap(char* filePath);
	
	bool loadMapAsPointList(char* fileName);
	
	bool loadSaphiraWld(char* fileName);
		
	bool loadVoronoi(char* filePath);

	bool loadPath(char* filePath);
	
	bool loadCarmen(char* filePath);

	bool loadBeeSoft(char* filePath);

	bool loadStageMap(char* filePath);
	bool saveStage(char* filePath,StageWorldOptions& options);
	
	bool loadMapViewerMap(char* filePath);

	bool loadRobotRun(char* filePath);
	bool loadImage(char* filePath);

	//return the coordinates in the actual map at the point (viewX,viewY)
	//in the window
	bool getMapCoords(long viewX, long viewY, long& mapX, long& mapY);

	//return the list of points in the map that are at the point (viewX,viewY) in the window
	//This is applicable when the map is zoomed out
	bool getMapCoordsSet(long viewX, long viewY, long& mapX1, long& mapY1, long& mapX2, long& mapY2);

	//Given a map coordinate, return the rectangle in the view window that is is drawn with
	inline bool getViewAreaFromMap(long mapX, long mapY, long& viewX1,long& viewY1,long& viewX2,long& viewY2);
	inline void getViewAreaFromMapStandardView(long mapX, long mapY, long& viewX1,long& viewY1,long& viewX2,long& viewY2);
	inline void getViewAreaFromMapUsingView(long mapX, long mapY, long& viewX1,long& viewY1,long& viewX2,long& viewY2);


	inline bool getViewAreaFromMap(long mapX1, long mapY1, long mapX2, long mapY2,
										long& viewX1,long& viewY1,long& viewX2,long& viewY2);
	inline void getViewAreaFromMapStandardView(long mapX1, long mapY1, long mapX2, long mapY2,
										long& viewX1,long& viewY1,long& viewX2,long& viewY2);
	inline void getViewAreaFromMapUsingView(long mapX1, long mapY1, long mapX2, long mapY2,
										long& viewX1,long& viewY1,long& viewX2,long& viewY2);

	bool getViewGridMap(){return _mapManager->getViewGridMap();}
	bool getViewVectorMap(){return _mapManager->getViewVectorMap();}

	//return true if the view window coordinate (viewX,viewY) is in the area used to display the map
	bool pointInViewArea(long viewX, long viewY);

	//set the subsection of the map to display
	void zoomIn(long minX, long minY, long maxX, long maxY);

	//zoom out of the map. The percentage specifies how far to zoom out.
	bool zoomOut(double percentage);	

	//change the size of the viewing window
	//if the view has changed because of the resize, return true. Otherwise return false
	bool resizeWindow(long newWidth, long newHeight);

	//recalculates all necessary variables
	void refresh();

	//makes the entire map fit in the window
	void resetView();

	bool undo()
	{
		bool retval = _mapManager->undo();
		if(retval)
			setAllScreenToChanged();
		return retval;
	}

	bool redo()
	{
		bool retval = _mapManager->redo();
		if(retval)
			setAllScreenToChanged();
		return retval;

	}

	//Paints the grid map, vectors, robot and ruler to the screen
	//The inplementation class of the ScreenPainterInterface interface
	//must call the draw() method on the object that it is passed in the
	//paintObject() method.
	bool repaint(IScreenPainter* painter);

	//set the Area of the map to be displayed
	//direction specifies which side of the map you refer to - NORTH,SOUTH,EAST,WEST
	void setDisplayedMapArea(int direction, long value);

    long getDisplayedMapArea(long direction){return getViewArea(direction);}
	
	long getViewArea(int direction);

	long getDisplayedMapWidth();

	long getDisplayedMapHeight();

	PointXYLong getWindowSize();

	long getSquareSize(){return _squareSize;}
		
	//now for the setter methods - these are used to manipulate the map

	//sets one or more points in the map based on an (x,y) position
	//in the view window
	void setPoint(long x, long y, float value);

	//sets a line based on window coordinates
	void setLine(long x1, long y1, long x2, long y2, float value);

	//returns the colour displayed at the (x,y) position in the window
	float getPointVal(long x, long y);

	//fills in a rectangle with the specified value based on two coordinates
	//in the window view
	void setRectangleFilled(long x1, long y1, long x2, long y2, float value);
	void setRectangle(long x1, long y1, long x2, long y2, float value);
	

	//fills an area in the map based on (X,Y) coordinates in the screen
	//this differs from the parent class' implementation, which takes mm as parameters
	void fillArea(long xVal, long yVal,float valueToSet,double tolerance);

	bool hasMap(){return _mapManager->hasMap();}
	bool hasPath(){return _mapManager->hasPath();}
	bool hasRobotRun(){return _mapManager->hasRobotRun();}
	bool hasVoronoi(){return _mapManager->hasVoronoi();}
	bool hasVectors(){return _mapManager->hasVectors();}
	bool hasDelaunay(){return _mapManager->hasDelaunay();}

	void setViewVectorMap(bool viewVectorMap)
	{
		_mapManager->setViewVectorMap(viewVectorMap);
		_finishedDrawingRuler = false;
	}
	
	void setViewRuler(bool viewRuler)
	{
		_rulerObj->setVisible(viewRuler);
		_finishedDrawingRuler = false;
	}

	void setViewVoronoi(bool viewVoronoi)
	{
		_viewVoronoi = viewVoronoi;
		_finishedDrawingRuler = false;	
	}

	void setViewDelaunay(bool viewDelaunay)
	{
		_viewDelaunay = viewDelaunay;
		_finishedDrawingRuler = false;	
	}

	void setViewVoronoiVertices(bool viewVoronoiVertices)
	{
		_viewVoronoiVertices = viewVoronoiVertices;
		_finishedDrawingRuler = false;	
	}	

	void setViewBackgroundGrid(bool viewBG)
	{
		_viewBackgroundGrid = viewBG;
		_finishedDrawingRuler = false;
	}

	void setViewRobot(bool val)
	{
		_viewRobot = val;
		_finishedDrawingRuler = false;
		_currentDrawingRobot.setVisible(val);
	}

	void setViewRobotRun(bool val)
	{
		_viewRobotRun = val;
		_finishedDrawingRuler = false;
	}

	bool getViewRobotRun()
	{
		return _viewRobotRun;
	}

	void setViewBorder(bool val)
	{
		_viewBorder = val;
		_finishedDrawingRuler = false;
	}

	
	void setViewGridMap(bool viewGridMap);

	void setViewPath(bool viewPath);

	bool getViewRobot()
	{
		return _viewRobot;
	}	

	bool getViewPath()
	{
		return _viewPath;
	}

	void setViewMinCellValue()
	{
		_viewCellMode = VIEW_CELL_MIN;
	}

	void setViewMaxCellValue()
	{
		_viewCellMode = VIEW_CELL_MAX;
	}

	void setViewAvgCellValue()
	{
		_viewCellMode = VIEW_CELL_AVG;
	}
	
	int getViewCellMode()
	{
		return _viewCellMode;
	}

	bool getViewRuler(){return _rulerObj->isVisible();}
	bool getViewVoronoi(){return _viewVoronoi;}
	bool getViewVoronoiVertices(){return _viewVoronoiVertices;}

	bool getViewDelaunay(){return _viewDelaunay;}

	bool getViewBackgroundGrid(){return _viewBackgroundGrid;}

	//the "clear" functions empty whatever list they refer to
	void clearVoronoi();
	void clearDelaunay();
	void clearPaths();
	void clearVectors();
	void clearGridMap();
	void clearRobotRun();
	bool clearRobots();
	
	void createNewLayer(){_mapManager->getNextLayer();}



	LineXYLong getChangedArea();
	
	bool saveBeeSoft(char* filePath){return _mapManager->saveBeeSoft(filePath);}
	bool saveCarmen(char* filePath){return _mapManager->saveCarmen(filePath);}
	bool saveMapAsPointList(char* filePath){return _mapManager->saveMapAsPointList(filePath);}
	bool saveMapViewerMap(char* filePath, bool saveGrid, bool saveVector)
	{
		return _mapManager->saveMapViewerMap(filePath,saveGrid,saveVector);
	}
	bool saveSaphiraWld(char* filePath){return _mapManager->saveSaphiraWld(filePath);}
	bool saveRossum(char* filePath){return _mapManager->saveRossum(filePath);}
	bool savePath(char* filePath){return _mapManager->savePath(filePath);}
	bool saveVoronoi(char* filePath){return _mapManager->saveVoronoi(filePath);}



	bool convertGridToLine(float threshold = 0.5);
	bool convertGridToLineWithVoronoi(float minThreshold, float maxThreshold, bool filterByCellValue = true, float value = 1);

	DrawableObject*   getDrawableRectangleFilled(PointXYLong pt1, PointXYLong pt2, DWORD colour= RGB(255,255,255));
	
	long getResolution(){return _mapManager->getResolution();}
	void setResolution(long res);

	//this sets the internal screen size to a size such that it matches the map 1:1
	//this is no good for displaying the map, it is for saving it as an image
	void matchScreenToMap();
	void unmatchScreenToMap();

	void setAllScreenToChanged();
	void setScreenAreaToChanged(long x1,long y1,long x2,long y2, int expand = 0);

	bool screenToGrid(long screenX,long screenY, float& gridX, float& gridY);
	bool gridToScreen(float gridX, float gridY, long& screenX, long& screenY);
	
	void mmToScreen(long mmX, long mmY,long& screenX, long& screenY);
	void screenToMm(long screenX, long screenY,long& mmX, long& mmY);
	void screenToMm(long screenX, long screenY,float& mmX, float& mmY);
	void screenDistToMmDist(long screenX,long screenY, long& mmX, long& mmY);

	void setBrushSize(int size){_brushSize = size;}

	bool averageGridMap(char* filePath);
	double correlateMap(char* filePath, bool popupAnswer);
	double mapScoreMap(char* filePath,bool justCompareOccAreas,  bool popupAnswer);
	bool generateCSpaceSimple(float min, float max, long distance);
	bool thresholdMap(float min, float max);
	bool smoothMap(float min, float max);
	
	//flip all values of the grid map. If a value was 0, it will be 1, 0.8 ->0.2 etc
	//Values of -1 stay the same
	bool negativeMap();

	bool translateMap(long xDist, long yDist);
	bool cropMap(long x1,long y1, long x2, long y2);

	//reset the iterator to display the entire map, starting at the top left corner
	bool resetIterator();

	//reset the iterator to display only the nodes that have changed
	//since the last paint
	bool resetIteratorOnlyChanged(int expand =0);

	//reset the iterator, but only the area of the map in the area defined by the two points
	//will be returned by the getNext() method
	bool resetIterator(long x1, long y1, long x2, long y2);

	enum
	{
		VIEW_CELL_AVG,
		VIEW_CELL_MAX,
		VIEW_CELL_MIN
	};
protected:
	MapViewManager(MapManager* childMapManager, WindowCallbackInterface* parentWindow = 0);
	MapViewManager(MapManager* childMapManager, GridMap<float> *m,WindowCallbackInterface* parentWindow = 0);

	virtual void		init();
	bool				calcSquareSize();
	
	void				addChangedArea(long x1, long y1, long x2, long y2, LineXYLong prevArea);
	void				addChangedArea(float x1, float y1, float x2, float y2, LineXYLong prevArea);
	PointXYLong			getViewSize();

	LineXYLayer			fixRobotSize(LineXYLayer object);
	
	void				disableRecordingChangedArea(){_changedAreaEnabled = false;}
	void				enableRecordingChangedArea(){_changedAreaEnabled = true;}
	
	bool				_changedAreaEnabled;
	DrawableObject*		createDrawableObject	(LineXYLayer&	object);
	bool				translateToScreenCoords(LineXYLayer& object);

	void				resetAllObjects();
	LineXY				LineXYLongToFloat(LineXYLong l, bool isRectangle);
	
	MapManager*						_mapManager;
	bool							_mapManagerFullyOwned;

	bool							_viewVoronoi;
	bool							_viewVoronoiVertices;

	bool							_viewDelaunay;

	bool							_viewPath;
	bool							_viewBackgroundGrid;
	bool							_viewRobotRun;
	bool							_viewRobot;
	bool							_viewBorder;
	int								_viewCellMode;

	bool							_finishedDrawingGrid;
	bool							_finishedDrawingBackgroundGrid;
	bool							_finishedDrawingRuler;	
	bool							_finishedDrawingVoronoi;
	bool							_finishedDrawingVoronoiVertices;
	
	bool							_finishedDrawingDelaunay;

	bool							_finishedDrawingPath;
	bool							_finishedDrawingRobotRun;

	bool							_drewBorder;
	bool							_clearedBackground;
	DWORD							_backgroundColour;

	DWORD							_gridColours[256];

	PointXYLong						_displayedArea;
	
	long							_mapMinX, _mapMaxX, _mapMinY,_mapMaxY;	
	
	long							_viewWidth, _viewHeight;
	long							_oldviewWidth, _oldviewHeight;

	long							_squareSize;
	bool							_rulerWasVisible;

	bool							_getMapFromView;
	int								_viewReduceFactor;
	int								_sampleStep;
	int								_mapAverageCount;

	
	long							_currentX, _currentY;
	long							_iteratorRightX, _iteratorLeftX, _iteratorTopY, _iteratorBottomY;

	
	bool							_drawOutSideMap;
	long							_layerWhenLastPathPlanned;

	DrawableRectangle				_currentRect;
	DrawableRectangleFilled			_currentFilledRect;
	DrawableLine					_currentLine;
	DrawableScanLine				_currentScanLine;
	DrawablePixel					_currentPixel;
	
	DrawableRobot					_currentDrawingRobot;
	
	DrawableObject*					_gridDrawObject;

	List<LineXYLayer>				_listGridBackground;//list of grid lines shown in background when vector map not shown

	DWORD							_vectorColour;
	DWORD							_voronoiColour;
	DWORD							_delaunayColour;
	DWORD							_pathColour;
	DWORD							_robotColour;

	long							_changedMaxX, _changedMinX, _changedMinY, _changedMaxY;
	bool							_changedValuesUnset;
	bool							_pathChanged,_gridChanged,_vectorsChanged,_voronoiChanged;

	//LineXYLong						_vectorBoundary;	

	VisRuler*						_rulerObj;
	
	long							_robotRadius;
	
	ListUnordered<ListUnordered<SosPose>* >		_listRobotRunLists;
	ListUnordered<SosPose> *					_currentRobotRunList;
	bool										_getNextRobotRunList;
	int											_currentRobotRunColourNum;
	DWORD										_currentRobotRunColour;
	Vector<DWORD>*								_robotRunColours;

	int								_brushSize;

	WindowCallbackInterface*		_parentWindow;

	bool							_mapViewerBusy;

	WorldOptions					_randomMapOptions;


	DEF_LOG
};


#endif