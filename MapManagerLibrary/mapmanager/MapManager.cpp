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


#include "../sosutil/SosUtil.h"
#include "../voronoi/VoronoiDiagramGenerator.h"
#include <fcntl.h>
#include "MapManager.h"

MapManager::MapManager()
{		
	init();
}

MapManager::MapManager(GridMap<float> *m)
{	
	init();
	addMap(m);
}
MapManager::MapManager(std::vector<LineXYLayer>* initialVectors, long resolution)
{
	init();

	setViewGridMap(false);
	setViewVectorMap(true);

	if(initialVectors == 0 || initialVectors->empty())
		return;

	LineXYLayer obj;

	float minX,maxX,minY,maxY;
	minX = maxX = minY = maxY = 0;

	int i = 0;
	//get the dimensions of the map
	for(i = 0; i < initialVectors->size(); i++)
	{
		//obj = initialVectors->at(i);
		obj = (*initialVectors)[i];

		if(i == 0)
		{
			minX = SosUtil::minVal(obj.pt1.x,obj.pt2.x);
			maxX = SosUtil::maxVal(obj.pt1.x,obj.pt2.x);
			minY = SosUtil::minVal(obj.pt1.y,obj.pt2.y);
			maxY = SosUtil::maxVal(obj.pt1.y,obj.pt2.y);
		}
		else
		{
			minX = SosUtil::minVal(minX,SosUtil::minVal(obj.pt1.x,obj.pt2.x));
			maxX = SosUtil::maxVal(maxX,SosUtil::maxVal(obj.pt1.x,obj.pt2.x));
			minY = SosUtil::minVal(minY,SosUtil::minVal(obj.pt1.y,obj.pt2.y));
			maxY = SosUtil::maxVal(maxY,SosUtil::maxVal(obj.pt1.y,obj.pt2.y));
		}
		
	}

	minX = minX / _resolution + 10;
	maxX = maxX / _resolution + 10;
	minY = minY / _resolution + 10;
	maxY = maxY / _resolution + 10;

	newMap(minX,maxX,minY,maxY);

	for(i = 0; i < initialVectors->size(); i++)
	{
		//obj = initialVectors->at(i);
		obj = (*initialVectors)[i];

		if(getViewGridMap())
		{
			obj.layer = getNextLayer();
		}
		else
		{
			obj.layer = getNextNegativeLayer();
		}
		obj.value = 1;

		setObject(obj);
	}

}

MapManager::~MapManager()
{
	LOG<<"In ~MapManager()"<<endl;

	resetAllObjects();

	LOG<<"~MapManager() after calling resetAllObjects";
	if(_myMap != 0)
	{
		LOG<<"Deleting _myMap = "<<_myMap;
		delete _myMap;
		_myMap = 0;
		
		LOG<<"After Deleting _myMap = "<<_myMap;
	}

	LOG<<"At end of MapManager destructor"<<endl;
}

void MapManager::init()
{
	GET_FILE_LOG
	//LOGGING_OFF
	ENTRYEXIT_LOG_OFF

	LOG<<"In MapManager::init()"<<endl;

	_myMap = 0;
	_mapMinX= _mapMaxX= _mapMinY=_mapMaxY=0;	
	
	_gotNegLayer = false;

	_gridChanged=_vectorsChanged=_voronoiChanged=true;
	
	_resolution = 100;//reduce grid map 10 times - 1 grid cell is 10 * 10 cm

	_mapAverageCount = 1;

	_hasError = false;
	_errorBuffer[0][0] = 0;
	_errorBuffer[1][0] = 0;


	_robotRadius = 1;

	_viewVoronoi			= false;
	_viewVoronoiVertices	= false;
	_viewBackgroundGrid		= true;
	_viewGridMap = true;
	_viewVectorMap = false;
	_viewPath = false;
	_viewRobotRun = false;

	_listObjects.setModeStack();		//it is important that this is done as a queue
	_listObjectsDeleted.setModeStack();
	_listObjectsUndone.setModeStack();
	_listObjectsReplaced.setModeStack();
	_listObjectsReplacedUndone.setModeStack();

	_listPoints.setModeStack();
	_listPointsUndone.setModeStack();

	_listUserActions.setModeStack();
	_listVoronoiLines.setModeQueue();	//queue or stack doesn't really matter here
	_listPathLines.setModeQueue();		//queue or stack doesn't really matter here

	_listUndoActions.setModeStack();
	_listUserActions.setModeStack();

	_listRobotRunLists.setModeQueue();
	
	//the first NUM_UNDO_STEPS layers are reserved for undo-operations.
	//all others are for objects
	_latestLayer = NUM_UNDO_STEPS; 
	_latestNegLayer = -2;

	_viewRobot = false;

}

void MapManager::resetAllObjects()
{
	LOG<<"In resetAllObjects()"<<endl;
	_listObjects.clear();

	resetUndoInfo();
	
	_listPoints.clear();

	_listVoronoiLines.clear();
	_listVoronoiVertices.clear();
	_listVoronoiEdges.clear();
	_listDelaunayLines.clear();
	_listPathGoalPoints.clear();
	_listPathLines.clear();
	_listObjects.clear();
	
	_gridLayer.reset();

	_latestLayer= 1;
	_latestNegLayer = -1;

	List<PointXYLong>* pointList = 0;
	LOG<<"Going to try to delete the path point lists, size "<<_pathLists.getListSize()<<endl;
	
	_pathLists.resetIterator();
	while(_pathLists.readNext(pointList))
	{
		LOG<<"Going to clear and delete the list "<<pointList<<endl;
		if(pointList != 0)
		{
			pointList->clear();
			delete pointList;
		}
	}
	
	_pathLists.clear();

	LOG<<"deleting the robot run lists, size = "<<_listRobotRunLists.getListSize()<<endl;

	ListUnordered<SosPose> * sensorReadingList = 0;
	while(_listRobotRunLists.popHead(sensorReadingList))
	{
		if(sensorReadingList != 0)
		{
			delete sensorReadingList;
		}
	}

	sensorReadingList = 0;
	
	_mapAverageCount = 1;	

	LOG<<"At the end of resetAllObjects"<<endl;

	LOGEXIT("resetAllObjects");
}

int MapManager::addMap(GridMap<float> *m, bool performShallowCopy)
{
	LOGENTRY("addMap")

	LOG<<"In addMap";
	long west = 0, east = 0, south = 0, north = 0;
	float val = 0;
	//if the map is not null, initialise the view to look at the complete map
	if(m != 0)
	{
		if(_myMap != 0)
			delete _myMap;

		m->getAllUpdatedDimensions(west,north,east,south);
		_myMap = new GridMap<float>(100,1,0);
		LOG<<"About to copy the map with dimensions west="<<west<<",east="<<east<<",north="<<north<<",south="<<south;
		LOGTIME;
		if(!performShallowCopy)
		{
			_myMap->copy(m);
			_myMap->setDimensions(west,north,east,south);
		}
		else
		{
			_myMap->clone(m);
		}
		
		resetAllObjects();
		
		_gridLayer.initFromMap(_myMap);
		
		_myMap->getAllUpdatedDimensions(_mapMinX,_mapMaxY,_mapMaxX,_mapMinY);

		_vectorBoundary.pt1.x = _gridLayer.getDimensions(WEST) * _resolution;
		_vectorBoundary.pt1.y = _gridLayer.getDimensions(SOUTH) * _resolution;
		_vectorBoundary.pt2.x = _gridLayer.getDimensions(EAST) * _resolution;
		_vectorBoundary.pt2.y = _gridLayer.getDimensions(NORTH) * _resolution;
		
	}

	_mapAverageCount = 1;

	LOGEXIT("addMap")
	return 0;
}

bool MapManager::newMap(long minX, long maxX, long minY, long maxY)
{
	resetAllObjects();

	LOG<<"newMap ("<<minX<<","<<minY<<") -> ("<<maxX<<","<<maxY<<")";
	if(_myMap == 0)
	{		
		_myMap = new GridMap<float>(1000,1,0);				
	}
	else
	{
		_myMap->reset();				
	}
	
	_mapAverageCount = 1;
	
	_myMap->setDimensions(minX/_resolution,maxY/_resolution,maxX/_resolution,minY/_resolution);

	_vectorBoundary.pt1.x = minX;
	_vectorBoundary.pt1.y = maxY;
	_vectorBoundary.pt2.x = maxX;
	_vectorBoundary.pt2.y = minY;
	
	_mapMinX = minX;
	_mapMaxX = maxX ;
	_mapMinY = minY;
	_mapMaxY = maxY;

	_listObjects.clear();
	_gridLayer.reset();
	_gridLayer.initFromMap(_myMap);

	_mapAverageCount = 1;

	return true;
}

bool MapManager::loadSaphiraWld(char* fileName)
{
	if(fileName == 0)
		return false;

	SaphiraWldParser parser;

	bool ret = parser.parseWldFile(fileName);

	if(!ret)//if the parser failed, either because it couldn't open the file or the file is incorrect
		return false;

	setViewGridMap(false);
	setViewVectorMap(true);

	resetAllObjects();

	LineXYLong line;
	LineXYLayer line2;

	long width = 0, height = 0;
	PointXYLong origin(0,0);
	
	PointXYZ robotPos(0,0,0);
	parser.getMapInfo(width,height,origin,robotPos);

	width = (width) + 10 * _resolution;
	height = (height) -10 * _resolution;
	//origin.x /= _resolution;
	//origin.y /= _resolution;

	long minX = origin.x - width /2 - 5;
	long maxX = origin.x + width /2 + 5;
	long minY = origin.y - height/2 -5;
	long maxY = origin.y + height/2 +5;

	//create a new map
	newMap(minX, maxX,minY,maxY);

	parser.resetIterator();

	bool firstVector = true;

	//go through all the lines and add each one to the list of objects
	while(parser.getNext(line))
	{
		line2.pt1.x = (float)line.pt1.x;
		line2.pt1.y = (float)line.pt1.y;
		line2.pt2.x = (float)line.pt2.x;
		line2.pt2.y = (float)line.pt2.y;

		line2.layer = getNextNegativeLayer();//no point putting into the grid map yet

		line2.type = OBJECT_TYPE_LINE;
		line2.value = 1; //make all lines black

		if(firstVector)
		{
			firstVector = false;
			_vectorBoundary.pt1.x = (long)SosUtil::minVal(line2.pt1.x,line2.pt2.x);
			_vectorBoundary.pt2.x = (long)SosUtil::maxVal(line2.pt1.x,line2.pt2.x);
			_vectorBoundary.pt1.y = (long)SosUtil::minVal(line2.pt1.y,line2.pt2.y);
			_vectorBoundary.pt2.y = (long)SosUtil::maxVal(line2.pt1.y,line2.pt2.y);

			LOG<<"Set the vectorBoundary to MinX = "<<_vectorBoundary.pt1.x <<",MinY = "<<_vectorBoundary.pt1.y<<",MaxX = "<<_vectorBoundary.pt2.x<<", MaxY = "<<_vectorBoundary.pt2.y;
		}

		setObject(line2);
	}
	
	if(parser.hasRobotPosition())
	{
		robotPos = parser.getRobotPosition();
		double robotWidth = (double)ROBOT_RADIUS;
		
		line2.pt1.x = robotPos.x - robotWidth;
		line2.pt1.y = robotPos.y + robotWidth;
		line2.pt2.x = robotPos.x + robotWidth;
		line2.pt2.y = robotPos.y - robotWidth;
		line2.type = OBJECT_TYPE_ROBOT;
		line2.layer = getNextNegativeLayer();
		line2.value = robotPos.value;
		setObject(line2);
	}

	_mapAverageCount = 1;

	LineXYLong temp = _vectorBoundary;

	SosUtil::ensureSmaller(temp.pt1.x,temp.pt2.x);
	SosUtil::ensureSmaller(temp.pt1.y,temp.pt2.y);

	//expand the boundary around the vectors by 10 cells
	_vectorBoundary.pt1.x = temp.pt1.x  - 10* _resolution;
	_vectorBoundary.pt2.x = temp.pt2.x + 10 * _resolution;
	_vectorBoundary.pt1.y = temp.pt1.y - 10 * _resolution;
	_vectorBoundary.pt2.y = temp.pt2.y + 10 * _resolution;

	copyVectorDimensionsToGrid();

	LOG<<"At end of loadSaphiraWld, vectorBoundary to MinX = "<<_vectorBoundary.pt1.x <<",MinY = "<<_vectorBoundary.pt1.y<<",MaxX = "<<_vectorBoundary.pt2.x<<", MaxY = "<<_vectorBoundary.pt2.y;
	LOG<<"_mapMinX = "<<_mapMinX<<",_mapMaxX = "<<_mapMaxX<<",_mapMinY = "<<_mapMinY<<", _mapMaxY = "<<_mapMaxY;

	return true;
}

void MapManager::copyVectorDimensionsToGrid()
{
	_gridLayer.setDimensions(_vectorBoundary.pt1.x/_resolution,
								_vectorBoundary.pt2.y/_resolution,
								_vectorBoundary.pt2.x/_resolution,
								_vectorBoundary.pt1.y/_resolution
							);
}

bool MapManager::hasRobot()
{
	_listObjects.resetIterator();

	LineXYLayer obj;

	while(_listObjects.readNext(obj))
	{
		if(obj.type == OBJECT_TYPE_ROBOT)
			return true;
	}

	return false;

}

bool MapManager::saveGridMap(char* fileName)
{
	GridMap<float> tempMap(100,1,-1);// = new GridMap<float>;

	getLatestGridMap(&tempMap);	

	return tempMap.save(fileName);
}

bool MapManager::saveSaphiraWld(char* fileName)
{
	if(fileName == 0)
		return false;
	
	ofstream out;
	out.open(fileName);

	if(!out.is_open())
		return false;

	long minX = SosUtil::minVal(_vectorBoundary.pt1.x,_vectorBoundary.pt2.x);
	long maxX = SosUtil::maxVal(_vectorBoundary.pt1.x,_vectorBoundary.pt2.x);
	long minY = SosUtil::minVal(_vectorBoundary.pt1.y,_vectorBoundary.pt2.y);
	long maxY = SosUtil::maxVal(_vectorBoundary.pt1.y,_vectorBoundary.pt2.y);

	const long EXTRA_BORDER = 1000;

	out<<";; "<<SosUtil::stripPath(fileName)<<endl;
	out<<";; Created with MapViewer application by Shane O'Sullivan"<<endl;

	long width = (maxX - minX +1)  + EXTRA_BORDER;
	long height = (maxY - minY +1)  + EXTRA_BORDER;
	long centreX = (minX ) - EXTRA_BORDER/2;
	long centreY = (minY ) - EXTRA_BORDER/2;
 
	out<<"width "<<width<<endl;
	out<<"height "<<height<<endl;
	out<<"origin ";
	out<<centreX<<' '<<centreY<<endl;

	_listObjects.resetIterator();

	LineXYLayer obj;
	LineXYLayer robotObject;
	bool robotFound = false;

	while(_listObjects.readNext(obj))
	{
		//first scan through the objects looking for a robot - jump out at the first one
		if(obj.type == OBJECT_TYPE_ROBOT)
		{
			robotFound = true;
			robotObject = obj;
			break;
		}
	}

	PointXY centre;
	if(robotFound)
	{
		centre.x = SosUtil::midWay(robotObject.pt1.x,robotObject.pt2.x);
		centre.y = SosUtil::midWay(robotObject.pt1.y,robotObject.pt2.y);

		//the angle the robot is facing is stored in the object's value
		out<<"position "<<long(centre.x)<<' '<<long(centre.y)<<" "<<(long)robotObject.value<<endl;
	}

	_listObjects.resetIterator();
	while(_listObjects.readNext(obj))
	{
		switch(obj.type)
		{
		case OBJECT_TYPE_LINE:
			//if the line is all inside one grid cell, output it as a single-cell rectangle.
			//otherwise output it as a line.... obviously enough
			if(obj.pt1 != obj.pt2)
			{
				out<<";; line"<<endl;
				
				out<<(long)obj.pt1.x<<" "<<(long)obj.pt1.y<<' '<<(long)obj.pt2.x<<' '<<(long)obj.pt2.y<<endl;
				break;
			}
		case OBJECT_TYPE_RECTANGLE:
			out<<";; rectangle"<<endl;
			SosUtil::ensureSmaller(obj.pt1.x,obj.pt2.x);
			SosUtil::ensureSmaller(obj.pt2.y,obj.pt1.y);

			out<<(long)obj.pt1.x<<' '<<(long)obj.pt1.y<<' '<<(long)obj.pt1.x<<' '<<(long)obj.pt2.y<<endl;//left line
			out<<(long)obj.pt1.x<<' '<<(long)obj.pt1.y<<' '<<(long)obj.pt2.x<<' '<<(long)obj.pt1.y<<endl;//top line
			out<<(long)obj.pt1.x<<' '<<(long)obj.pt2.y<<' '<<(long)obj.pt2.x<<' '<<(long)obj.pt2.y<<endl;//bottom line
			out<<(long)obj.pt2.x<<' '<<(long)obj.pt1.y<<' '<<(long)obj.pt2.x<<' '<<(long)obj.pt2.y<<endl;//right line

			break;
		case OBJECT_TYPE_RECTANGLE_FILLED:
			out<<";; rectangle - originally filled, now empty"<<endl;

			SosUtil::ensureSmaller(obj.pt1.x,obj.pt2.x);
			SosUtil::ensureSmaller(obj.pt2.y,obj.pt1.y);

			out<<(long)obj.pt1.x<<' '<<(long)obj.pt1.y<<' '<<(long)obj.pt1.x<<' '<<(long)obj.pt2.y<<endl;//left line
			out<<(long)obj.pt1.x<<' '<<(long)obj.pt1.y<<' '<<(long)obj.pt2.x<<' '<<(long)obj.pt1.y<<endl;//top line
			out<<(long)obj.pt1.x<<' '<<(long)obj.pt2.y<<' '<<(long)obj.pt2.x<<' '<<(long)obj.pt2.y<<endl;//bottom line
			out<<(long)obj.pt2.x<<' '<<(long)obj.pt1.y<<' '<<(long)obj.pt2.x<<' '<<(long)obj.pt2.y<<endl;//right line

			break;
		}

	}

	out.close();

	return true;
}


//saving the voronoi map takes the following form:
//voronoi
//lines
//x1 y1 x2 y2
//x1 y1 x2 y2
//........
//edges
//x1 y1 x2 y2
//x1 y1 x2 y2
//........
//vertices
//x1 y1
//x1 y1
//.....etc
bool MapManager::saveVoronoi(char* fileName)
{
	if(fileName == 0 || (_listVoronoiLines.getListSize() == 0 && _listDelaunayLines.getListSize() == 0))
	{
		LOG<<"MapManager::saveVoronoi returning false because _listVoronoiLines.getListSize() = "<<_listVoronoiLines.getListSize()<<" and _listDelaunayLines.getListSize() = "<<_listDelaunayLines.getListSize();
		return false;
	}
	
	ofstream out;
	
	LineXY line;
	PointXY pt;

	out.open(fileName);

	if(!out.is_open())
	{
		LOG<<"Failed to open the file "<<fileName;
		return false;
	}
	out<<"voronoi\n";//the first entry in a voronoi 

	_listVoronoiLines.resetIterator();

	if(_listVoronoiLines.getListSize() > 0)
	{		
		out<<"lines\n"; //the second entry in a voronoi 

		while(_listVoronoiLines.readNext(line))
		{
			mmToGrid(line.pt1.x,line.pt1.y,line.pt1.x,line.pt1.y);
			mmToGrid(line.pt2.x,line.pt2.y,line.pt2.x,line.pt2.y);
			out<<line.pt1.x<<' '<<line.pt1.y<<' '<<line.pt2.x<<' '<<line.pt2.y<<endl;
		}
	}

	_listVoronoiEdges.resetIterator();

	if(_listVoronoiEdges.getListSize() > 0)
	{
		out<<"edges\n";
		while(_listVoronoiEdges.readNext(line))
		{
			out<<line.pt1.x<<' '<<line.pt1.y<<' '<<line.pt2.x<<' '<<line.pt2.y<<endl;
		}
	}

	_listVoronoiVertices.resetIterator();

	if(_listVoronoiVertices.getListSize() > 0)
	{
		out<<"vertices\n";
		while(_listVoronoiVertices.readNext(pt))
		{
			out<<pt.x<<' '<<pt.y<<endl;
		}
	}

	if(_listDelaunayLines.getListSize() > 0)
	{
		_listDelaunayLines.resetIterator();
		out<<"delaunaylines"<<endl;

		while(_listDelaunayLines.readNext(line))
		{
			mmToGrid(line.pt1.x,line.pt1.y,line.pt1.x,line.pt1.y);
			mmToGrid(line.pt2.x,line.pt2.y,line.pt2.x,line.pt2.y);
			out<<line.pt1.x<<' '<<line.pt1.y<<' '<<line.pt2.x<<' '<<line.pt2.y<<endl;
		}
	}

	out.close();
	return true;
}

bool MapManager::loadVoronoi(char* fileName)
{
	if(fileName == 0 )
		return false;

	LOG<<"In loadVoronoi";
	const int TYPE_LINE = 1, TYPE_EDGE = 2, TYPE_VERTEX = 3, TYPE_DELLINE = 4;
	int entryType = 0;

	ifstream in;
	in.open(fileName);

	if(!in.is_open())
	{
		LOG<<"Couldn't open the file, returning false";
		return false;
	}
	bool fileIsIncorrect = false;

	char buffer1[250]={0}, buffer2[250]={0},buffer3[250]={0},buffer4[250]={0};
	in>>buffer1;

	//the first entry in a voronoi diagram file must be the word "voronoi"
	if(stricmp(buffer1,"voronoi") != 0)
	{	
		LOG<<"The first token is wrong = '"<<buffer1<<"' so returning false";
		in.close();
		return false;
	}

	float minX = 0,maxX = 0,minY = 0,maxY = 0;
	bool boundsSet = false;

	LineXY line;
	PointXY point;
	ListUnordered<LineXY> * listPtr = 0;

	ListUnordered<LineXY> listVoronoiLines,listVoronoiEdges, listDelaunayLines;
	ListUnordered<PointXY> listVoronoiVertices;

	_listVoronoiLines.clear();
	_listVoronoiEdges.clear();
	_listVoronoiVertices.clear();

	in>>buffer1;

	if(stricmp(buffer1,"edges") == 0)
	{
		entryType = TYPE_EDGE;
		listPtr = &listVoronoiEdges;
	}
	else if(stricmp(buffer1,"lines") == 0)
	{
		entryType = TYPE_LINE;
		listPtr = &listVoronoiLines;
	}
	else if(stricmp(buffer1,"vertices") == 0)
	{
		entryType = TYPE_VERTEX;
		listPtr = 0;
	}
	else if(stricmp(buffer1,"delaunaylines") == 0)
	{
		entryType = TYPE_DELLINE;
		listPtr = &listDelaunayLines;		
	}

	if(entryType == 0)
	{
		LOG<<"The first token was incorrect, it was '"<<buffer1<<"'";
		in.close();
		return false;
	}

	while(!in.eof() && !fileIsIncorrect)
	{
		if(entryType == TYPE_EDGE || entryType == TYPE_LINE || entryType == TYPE_DELLINE)
		{
			in>>buffer1;

			while(!in.eof() && !fileIsIncorrect)
			{			
				if(!SosUtil::is_numeric(buffer1))
				{
					if(stricmp(buffer1,"lines") == 0)
					{
						entryType = TYPE_LINE;
						listPtr = &listVoronoiLines;
						break;
					}
					else if(stricmp(buffer1,"edges") == 0)
					{
						entryType = TYPE_EDGE;
						listPtr = &listVoronoiEdges;
						break;
					}
					else if(stricmp(buffer1,"vertices") == 0)
					{
						entryType = TYPE_VERTEX;
						break;
					}					
					else if(stricmp(buffer1,"delaunaylines") == 0)
					{
						entryType = TYPE_DELLINE;
						listPtr = &listDelaunayLines;
						break;
					}
					else
					{
						LOG<<"1.Setting fileIsIncorrect because "<<buffer1<<" is not correct";
						fileIsIncorrect = true;
						break;
					}
				}

				in>>buffer2;
				
				LOG<<"buffer2 = "<<buffer2;
	
				if(in.eof()) break;

				if(!SosUtil::is_numeric(buffer2))
				{
					if(stricmp(buffer2,"lines") == 0)
					{
						entryType = TYPE_LINE;
						listPtr = &listVoronoiLines;
						break;
					}
					else if(stricmp(buffer2,"edges") == 0)
					{
						entryType = TYPE_EDGE;
						listPtr = &listVoronoiEdges;
						break;
					}
					else if(stricmp(buffer2,"vertices") == 0)
					{
						entryType = TYPE_VERTEX;
						break;
					}
					else if(stricmp(buffer2,"delaunaylines") == 0)
					{
						entryType = TYPE_DELLINE;
						listPtr = &listDelaunayLines;		
						break;
					}
					else
					{						
						LOG<<"2.Setting fileIsIncorrect because "<<buffer2<<" is not correct";
						fileIsIncorrect = true;
						break;
					}
				}

				in>>buffer3;
				LOG<<"buffer3 = "<<buffer3;

				if(in.eof()) break;

				if(!SosUtil::is_numeric(buffer3))
				{
					if(stricmp(buffer3,"lines") == 0)
					{
						entryType = TYPE_LINE;
						listPtr = &listVoronoiLines;
						break;
					}
					else if(stricmp(buffer3,"edges") == 0)
					{
						entryType = TYPE_EDGE;
						listPtr = &listVoronoiEdges;
						break;
					}
					else if(stricmp(buffer3,"vertices") == 0)
					{
						entryType = TYPE_VERTEX;
						break;
					}
					else if(stricmp(buffer3,"delaunaylines") == 0)
					{
						entryType = TYPE_DELLINE;
						listPtr = &listDelaunayLines;		
						break;
					}
					else
					{
						LOG<<"3.Setting fileIsIncorrect because "<<buffer3<<" is not correct";
						fileIsIncorrect = true;
						break;
					}
				}

				in>>buffer4;
				if(in.eof()) break;

				if(!SosUtil::is_numeric(buffer4))
				{
					if(stricmp(buffer4,"lines") == 0)
					{
						entryType = TYPE_LINE;
						listPtr = &listVoronoiLines;
						break;
					}
					else if(stricmp(buffer4,"edges") == 0)
					{
						entryType = TYPE_EDGE;
						listPtr = &listVoronoiEdges;
						break;
					}
					else if(stricmp(buffer4,"vertices") == 0)
					{
						entryType = TYPE_VERTEX;
						break;
					}
					else if(stricmp(buffer4,"delaunaylines") == 0)
					{
						entryType = TYPE_DELLINE;
						listPtr = &listDelaunayLines;		
						break;
					}
					else
					{
						LOG<<"4.Setting fileIsIncorrect because "<<buffer1<<" is not correct";
						fileIsIncorrect = true;
						break;
					}
				}		

				line.pt1.x = (float)atof(buffer1);
				line.pt1.y = (float)atof(buffer2);
				line.pt2.x = (float)atof(buffer3);
				line.pt2.y = (float)atof(buffer4);

				if(!boundsSet)
				{
					minX = SosUtil::minVal(line.pt1.x,line.pt2.x);
					maxX = SosUtil::maxVal(line.pt1.x,line.pt2.x);
					minY = SosUtil::minVal(line.pt1.y,line.pt2.y);
					maxY = SosUtil::maxVal(line.pt1.y,line.pt2.y);
					boundsSet = true;
				}
				else
				{
					minX = SosUtil::minVal(minX,SosUtil::minVal(line.pt1.x,line.pt2.x));
					maxX = SosUtil::maxVal(maxX,SosUtil::maxVal(line.pt1.x,line.pt2.x));
					minY = SosUtil::minVal(minY,SosUtil::minVal(line.pt1.y,line.pt2.y));
					maxY = SosUtil::maxVal(maxY,SosUtil::maxVal(line.pt1.y,line.pt2.y));
				}

				if(listPtr == &listVoronoiLines || listPtr == &listDelaunayLines)
				{
					long x,y;
					gridToMm(line.pt1.x,line.pt1.y,x,y);
					line.pt1.x = (float)x;
					line.pt1.y = (float)y;
					gridToMm(line.pt2.x,line.pt2.y,x,y);
					line.pt2.x = (float)x;
					line.pt2.y = (float)y;
					
				}
				
				listPtr->push(line);				
				
				in>>buffer1;
			}
		}
		else 
		{
			in>>buffer1;
			LOG<<"buffer1 = "<<buffer1;
		
			while(!in.eof() && !fileIsIncorrect)
			{
				if(!SosUtil::is_numeric(buffer1))
				{
					if(stricmp(buffer1,"lines") == 0)
					{
						entryType = TYPE_LINE;
						listPtr = &listVoronoiLines;
						break;
					}
					else if(stricmp(buffer1,"edges") == 0)
					{
						entryType = TYPE_EDGE;
						listPtr = &listVoronoiEdges;
						break;
					}
					else if(stricmp(buffer1,"vertices") == 0)
					{
						entryType = TYPE_VERTEX;
						break;
					}
					else if(stricmp(buffer1,"delaunaylines") == 0)
					{
						entryType = TYPE_DELLINE;
						listPtr = &listDelaunayLines;	
						break;
					}
					else
					{
						LOG<<"5.Setting fileIsIncorrect because "<<buffer1<<" is not correct";
						fileIsIncorrect = true;
						break;
					}
				}

				in>>buffer2;
				if(in.eof()) break;

				if(!SosUtil::is_numeric(buffer2))
				{
					if(stricmp(buffer2,"lines") == 0)
					{
						entryType = TYPE_LINE;
						listPtr = &listVoronoiLines;
						break;
					}
					else if(stricmp(buffer2,"edges") == 0)
					{
						entryType = TYPE_EDGE;
						listPtr = &listVoronoiEdges;
						break;
					}
					else if(stricmp(buffer2,"delaunaylines") == 0)
					{
						entryType = TYPE_DELLINE;
						listPtr = &listDelaunayLines;	
						break;
					}
					else if(stricmp(buffer2,"vertices") == 0)
					{
						entryType = TYPE_VERTEX;
						break;
					}
					else
					{
						LOG<<"6.Setting fileIsIncorrect because "<<buffer2<<" is not correct";
						fileIsIncorrect = true;
						break;
					}
				}
				point.x = (float)atof(buffer1);
				point.y = (float)atof(buffer2);

				if(!boundsSet)
				{
					minX = point.x;
					maxX = point.x;
					minY = point.y;
					maxY = point.y;
					boundsSet = true;
				}
				else
				{
					minX = SosUtil::minVal(minX,point.x);
					maxX = SosUtil::maxVal(maxX,point.x);
					minY = SosUtil::minVal(minY,point.y);
					maxY = SosUtil::maxVal(maxY,point.y);
				}
				
				listVoronoiVertices.push(point);				
				
				in>>buffer1;
				
			}
		}
	}

	in.close();

	//if the file had an error in it, clear all the lists and return false
	if(fileIsIncorrect)
	{
		listVoronoiVertices.clear();
		listVoronoiEdges.clear();
		listVoronoiLines.clear();
		listDelaunayLines.clear();
		return false;
	}

	long mmX1, mmX2, mmY1,mmY2;
	
	gridToMm(minX,minY, mmX1,mmY1);
	gridToMm(maxX,maxY, mmX2,mmY2);

	if(!hasMap())
	{
		newMap(mmX1,mmX2,mmY1,mmY2);
	}
	else
	{
		_vectorBoundary.pt1.x = SosUtil::minVal(_vectorBoundary.pt1.x,mmX1);
		_vectorBoundary.pt2.x = SosUtil::maxVal(_vectorBoundary.pt2.x,mmX2);
		_vectorBoundary.pt1.y = SosUtil::minVal(_vectorBoundary.pt1.y,mmY1);
		_vectorBoundary.pt2.y = SosUtil::maxVal(_vectorBoundary.pt2.y,mmY2);
	}

	LineXY obj;
	PointXY ptObj;
	long x1, y1, x2, y2;
	x1 = y1 = x2 = y2 = 0;

	while(listVoronoiVertices.popHead(ptObj))
	{		
		_listVoronoiVertices.push(ptObj);
	}
	while(listVoronoiEdges.popHead(obj))
	{
		_listVoronoiEdges.push(obj);
	}
	while(listVoronoiLines.popHead(obj))
	{
		_listVoronoiLines.push(obj);
	}
	while(listDelaunayLines.popHead(obj))
	{
		_listDelaunayLines.push(obj);
	}

	LOG<<"Got "<<_listDelaunayLines.getListSize()<<" delaunay lines";
	LOG<<"Got "<<_listVoronoiLines.getListSize()<<" voronoi lines";

	_viewVoronoi = true;

	LOG<<"load voronoi returning true";
	return true;
}

bool MapManager::saveMapAsPointList(char* fileName)
{
	if(fileName == 0 || !hasMap())
		return false;

	long west = 0,east = 0,south = 0,north = 0;

	_gridLayer.getDimensions(west,north,east,south);

	ofstream out;
	out.open(fileName);
	if(!out.is_open())
		return false;

	out<<"gridpointlist\n";
	out<<"width "<<east - west + 1<<endl;
	out<<"height "<<north - south + 1<<endl;

	for(long x = west; x <= east; x++)
	{
		for(long y = south; y<= north; y++)
		{
			out<<x<<' '<<y<<' '<<_gridLayer.read(x,y)<<endl;
		}
	}

	out.close();

	return true;
}

//Loads a map as a list of points.  The format is as follows:
//
//gridpointlist
//width 154
//height 203
//0 0 0.5
//0 1 0.6 
//......etc 
bool MapManager::loadMapAsPointList(char* fileName)
{
	long width = 0, height = 0;
	bool isWidth = false, isHeight = false;
	GridMap<float>* tempMap=0;

	if(fileName == 0)
		return false;

	ifstream in;
	in.open(fileName);

	if(!in.is_open())
		return false;

	char buffer[256];

	in>>buffer;

	if(stricmp(buffer,"gridpointlist") != 0)
	{
		LOG<<"Loading point list - didn't find gridpointlist";
		setErrorStrings("Error","File must start with 'gridpointlist'");
		in.close();
		return false;
	}

	in>>buffer;
	if(stricmp(buffer,"width") == 0)
	{
		isWidth = true;
	}
	else if(stricmp(buffer,"height") == 0)
	{
		isHeight = true;
	}
	else
	{
		in.close();
		return false;
	}

	in>>buffer; 
	if(!SosUtil::is_numeric(buffer))
	{
		in.close();
		return false;
	}
	if(isWidth)
	{
		width = atol(buffer);
	}
	else if(isHeight)
	{
		height = atol(buffer);
	}

	in>>buffer;
	if(stricmp(buffer,"width") == 0 && !isWidth)//if we've already gotten the width, getting a 2nd time is an error
	{
		isHeight = false;
		isWidth = true;
	}
	else if(stricmp(buffer,"height") == 0 && !isHeight)
	{
		isWidth = false;
		isHeight = true;
	}
	else
	{
		in.close();
		return false;
	}

	in>>buffer; 
	if(!SosUtil::is_numeric(buffer))
	{
		in.close();
		return false;
	}
	if(isWidth)
	{
		width = atol(buffer);
	}
	else if(isHeight)
	{
		height = atol(buffer);
	}

	char buffer2[256]={0},buffer3[256]={0};
	long x = 0, y = 0;
	float val = 0;
	
	in>>buffer;

	tempMap = new GridMap<float>(100,1,0);
	float max = 0;

	long maxX, minX,maxY,minY;
	bool boundsUnset = true;
	

	LOG<<"Loading point list - about to get all the numbers, width = "<<width<<", height = "<<height;
	while(!in.eof())
	{
		in>>buffer2;
		in>>buffer3;

		if(in.eof())
			continue;

		//only numeric strings are allowed from now on
		if(!SosUtil::is_numeric(buffer) || !SosUtil::is_numeric(buffer2) || !SosUtil::is_numeric(buffer3))
		{
			delete tempMap;
			in.close();
			setErrorStrings("Error","Tokens must be numeric");
			return false;
		}

		x = atol(buffer);
		y = atol(buffer2);
		val = (float)atof(buffer3);

		if(boundsUnset)
		{
			minX = maxX = x;
			minY = maxY = y;
			boundsUnset = false;
		}
		else
		{
			minX = SosUtil::minVal(minX,x);
			maxX = SosUtil::maxVal(maxX,x);
			minY = SosUtil::minVal(minY,y);
			maxY = SosUtil::maxVal(maxY,y);			
		}

		tempMap->updateGridRef(val,x,y);
		
		in>>buffer;
	}

	tempMap->setDimensions(minX,maxY,maxX,minY);

	addMap(tempMap,true);
	
	in.close();

	return true;

}

bool MapManager::loadGridMap(char* filePath)
{
	//first try to load it as a MapViewerMap
	if(loadMapViewerMap(filePath))
	{
		return true;
	}

	//GridMapParser will try to parse all known types of grid map, and copy the map into the
	//GridMap object that is passed as the second parameter (as a reference of course)
	GridMapParser parser;
	GridMap<float> newGridMap(1000,0,0);

	if(!parser.parseFile(filePath,&newGridMap))
	{
		setErrorStrings("Error!","Failed to parse the grid map file");
		return false;
	}

	addMap(&newGridMap,true);

	setViewGridMap(true);
	setViewVectorMap(false);

	return true;
}

bool MapManager::loadCarmen(char* filePath)
{
	if(filePath == 0 || strlen(filePath) == 0)
	{
		LOG<<"filePath is NULL";
		setErrorStrings("Error!","No file specified");
		return false;
	}

	GridMap<float> tempMap(1000,1,0);

	bool retval = CarmenTranslator::loadCarmenMap(filePath,&tempMap);

	if(!retval)
	{
		setErrorStrings("Error!","Failed to parse the grid map file");
		return false;
	}

	addMap(&tempMap);

	return true;
}

bool MapManager::saveCarmen(char* filePath)
{
	if(!hasMap())
	{
		setErrorStrings("Error!","No map loaded");		
		return false;
	}
	if(filePath == 0 || strlen(filePath) == 0)
	{
		setErrorStrings("Error!","No File Name Specified");
		return false;
	}

	GridMap<float> tempMap(1000,1,0);

	getLatestGridMap(&tempMap);

	bool retval = CarmenTranslator::saveCarmenMap(filePath,&tempMap,1.0/_resolution);

	if(!retval)
	{
		setErrorStrings("Error!","Failed to save the grid map file");
		return false;
	}

	return true;
}

bool MapManager::loadBeeSoft(char* filePath)
{
	if(filePath == 0 || strlen(filePath) == 0)
	{
		LOG<<"filePath is NULL";
		setErrorStrings("Error!","No file name specifiel");
		return false;
	}

	GridMap<float> tempMap(1000,1,0);

	bool retval = CarmenTranslator::loadBeesoftMap(filePath,&tempMap);

	if(!retval)
	{
		setErrorStrings("Error!","Failed to parse the grid map file");
		return false;
	}

	addMap(&tempMap);

	return true;
}

bool MapManager::saveBeeSoft(char* filePath)
{
	if(!hasMap())
	{
		setErrorStrings("Error!","No map loaded");		
		return false;
	}
	if(filePath == 0 || strlen(filePath) == 0)
	{
		setErrorStrings("Error!","No File Name Specified");
		return false;
	}
	
	GridMap<float> tempMap(1000,1,0);

	getLatestGridMap(&tempMap);

	bool retval = CarmenTranslator::saveBeeSoftMap(filePath,&tempMap,(1.0/_resolution) * 100);

	if(!retval)
		return false;

	return true;
}

//Only PNM images are supported right now
bool MapManager::loadImage(char* filePath)
{
	if(filePath == 0 || strlen(filePath) == 0)
		return false;
	
	char token[255];
	int whiteNum;
	const int LINE_LENGTH = 1024;
	unsigned int width = 0, height = 0;

	ifstream in;

	in.open(filePath);

	if( !in.is_open())
	{
		return false;
	}

	in>>token;

	if( stricmp(token, "P5" ) != 0 )
	{
		return false; 
	}

	// ignore the end of this line
	in.ignore( LINE_LENGTH, '\n' );

	// ignore comment lines
	while(in.peek() == '#')
	{
		in.ignore( LINE_LENGTH, '\n' );  
	}

	in>>width>>height>>whiteNum;

	in.ignore( LINE_LENGTH, '\n' );

	unsigned char *data = new unsigned char[ width * height ];

	in.read( (char*)data, width * height );

	if( (unsigned int)(in.gcount()) != width * height )
	{
		in.close();
		delete[] data;
		return false;
	}

	GridMap<float> newMap(1000,0,0);
	
	for(long x = 0; x< width; x++)
	{
		for(long y = 0; y< height; y++)
		{
			if(data[x + (y*width)] > 0)
				newMap.updateGridRef(1,x,y);
			else
				newMap.updateGridRef(0,x,y);
		}
	}

	addMap(&newMap,true);

	in.close();
	delete[] data;

	return true;
	



	return true;
}

bool MapManager::loadStageMap(char* filePath)
{
	LOG<<"loadStageMap("<<filePath<<")";
	StageWorldFileParser parser;

	const char POSE_STRING[]		= "pose";
	const char SIZE_STRING[]		= "size";
	const char RESOLUTION_STRING[]	= "resolution";
	const char SCALE_STRING[]		= "scale";
	const char ORIGIN_STRING[]		= "origin";

	int numBitmaps = 0;

	//if we can't parse the file, then return false;
	if(!parser.loadFile(filePath))
	{
		setErrorStrings(0,ERROR_FILE_OPEN(filePath));
		return false;
	}

	resetAllObjects();

	char imageFileName[512] = {0};

	//first get the units used - if none are found, then metres are used
	float resolutionDivisor = 1, resolution = 0.1f;
	float temp = 0;
	long numEntities = parser.numEntities();

	bool useRadians = false;
	PointXY origin;

	int i = 0;
	ListUnordered<LineXYLayer> objects;
	objects.setModeQueue();
	LineXYLayer obj;

	ListUnordered<LineXYLayer> boxes;
	boxes.setModeQueue();

	LineXYLayer box;

	float errorValue = (float)847463.2222233445564333;
	float poseX = 0, poseY = 0, poseAngle = 0;
	float sizeX = 0, sizeY = 0;
	
	for(i = 0; i< numEntities; i++)
	{
		const char *type = parser.entityType(i);

		//if there is no type, it could be resolution or unit_length
		if(type == 0 || strlen(type) == 0)
		{
			const char* val = parser.getStr(i,"unit_length","m");

			if(val != 0)
			{
				if(stricmp(val,"mm") == 0)
				{
					resolutionDivisor = 1;					
				}
				else if(stricmp(val,"cm") == 0)
				{
					resolutionDivisor = 10;
				}
				else if(stricmp(val,"meters") == 0 || stricmp(val,"metres"))
				{
					resolutionDivisor = 1000;					
				}
			}
	
			const char* unitAngle = parser.getStr(i,"unit_angle",0);

			if(unitAngle != 0 && strcmp(unitAngle,"radians") == 0)
			{
				useRadians = true;
			}
		
			temp = parser.getDbl(i, RESOLUTION_STRING,-1);

			if(temp != -1)
			{
				resolution = temp;
			}
		}
		//process the robots
		else if(stricmp(type,"position") == 0 || stricmp(type,"omniposition") == 0)
		{
			poseX = parser.getDbl(i,POSE_STRING,0,errorValue);
			poseY = parser.getDbl(i,POSE_STRING,1,errorValue);
			poseAngle = parser.getDbl(i,POSE_STRING,2,errorValue);
	
			//if we couldn't get any of the robot pose values, skip this entity
			if(poseX == errorValue || poseY == errorValue || poseAngle == errorValue)
			{
				continue;
			}

			obj.pt1.x = poseX;
			obj.pt2.x = poseX;
			obj.pt1.y = poseY;
			obj.pt2.y = poseY;
			obj.value = poseAngle;
		 	objects.push(obj);
		}
		else if(stricmp(type,"bitmap") == 0 || stricmp(type,"environment") == 0)
		{
			//we only support one bitmap object, so check that we haven't already
			//read one.  If we have, display an error message and return.
			if(numBitmaps != 0)
			{
				setErrorStrings(STD_ERROR_TITLE,STD_ONE_BITMAP_SUPPORTED);
				return false;
			}
			numBitmaps++;

			const char* value = parser.getStr(i,"file",0);

			if(value != 0)
			{
				strcpy(imageFileName,value);
				LOG<<"Got the image filename: "<<imageFileName;

				if(SosUtil::endsWith(imageFileName,"gz"))
				{
					setErrorStrings(STD_ERROR_TITLE,STD_GZ_NOT_SUPPORTED);
					return false;

				}
			}

			float temp = parser.getDbl(i,RESOLUTION_STRING,errorValue);

			if(temp != errorValue)
			{
				LOG<<"Got the internal bitmap resolution "<<temp;
				resolution = temp;//overwrite the resolution with this resolution
			}

			temp = parser.getDbl(i,SCALE_STRING,errorValue);
			if(temp != errorValue)
			{
				LOG<<"Got the internal bitmap scale "<<temp;
				resolution = temp;//overwrite the resolution with this resolution
			}
		}
		else if(stricmp(type,"box") == 0)
		{
			LOG<<"Found a BOX entity";

			poseX = parser.getDbl(i,POSE_STRING,0,errorValue);
			poseY = parser.getDbl(i,POSE_STRING,1,errorValue);
			poseAngle = parser.getDbl(i,POSE_STRING,2,errorValue);

			//if we couldn't get any of the robot pose values, skip this entity
			if(poseX == errorValue || poseY == errorValue || poseAngle == errorValue)
			{
				LOG<<"Error, couldn't get all robot pose values";
				LOG<<"poseX = "<<poseX<<", poseY = "<<poseY<<", poseAngle = "<<poseAngle<<", and the errorValue = "<<errorValue;
				continue;
			}

			//pt1 is the bottom left of the map
			//pt2 is the top right
			box.pt1.x = poseX ;
			box.pt2.x = poseX ;
			box.pt1.y = poseY ;
			box.pt2.y = poseY ;
			box.value = poseAngle;

			boxes.push(box);

			LOG<<"Got a box with corners ("<<box.pt1.x<<","<<box.pt1.y<<") -> ("<<box.pt2.x<<","<<box.pt2.y<<") at rotation "<<box.value;
		}
		else if(stricmp(type,"gui") == 0)
		{
			origin.x = parser.getDbl(i,ORIGIN_STRING, 0,errorValue);
			origin.y = parser.getDbl(i,ORIGIN_STRING, 1,errorValue);

			if(origin.x == errorValue || origin.y == errorValue)
			{
				origin.x = origin.y = 0;
				continue;
			}
			
		}
	}

	if(strlen(imageFileName) < 1)
	{
		setErrorStrings(0,STD_NO_IMAGE_FILE_NAME);
		return false;
	}

	LOG<<"Have a resolutionDivisor = "<<resolutionDivisor<<" and resolution "<<resolution;
	char dirPath[512] = {0};
	SosUtil::getPath(filePath,dirPath);

	char pnmFullPath[512] = {0};
	strcpy(pnmFullPath,dirPath);
	if(!SosUtil::endsWith(pnmFullPath,"/") && !SosUtil::endsWith(pnmFullPath,"\\"))
	{
		strcat(pnmFullPath,"/");
	}

	strcat(pnmFullPath,imageFileName);

	LOG<<"Going to try to open image "<<pnmFullPath;

	//load the PNM image.
	if(!loadImage(pnmFullPath))
	{
		setErrorStrings(STD_ERROR_TITLE,ERROR_FILE_OPEN(pnmFullPath));
		return false;
	}

	//now multiply the resolution by the resolutionDivisor, which converts from 
	//cm and metres to mm
	_resolution = long((float)resolution * (float)resolutionDivisor + 0.5f) ;

	//Now go through all the robots and fix their size
	//All robots must be ROBOT_RADIUS in radius
	double robotWidth = (double)ROBOT_RADIUS;

	origin.x *= (float)resolutionDivisor;
	origin.y *= (float)resolutionDivisor;

	if(objects.getListSize() > 0)
	{
		//make sure that the robots are visible if they exist;
//		setViewRobot(true);
		objects.resetIterator();

		while(objects.readNext(obj))
		{
			LOG<<"Robot centre was ("<<obj.pt1.x<<","<<obj.pt2.y<<") and is now ";
			obj.pt1.x *= (float)resolutionDivisor;
			obj.pt1.y *= (float)resolutionDivisor;
			obj.pt2.x *= (float)resolutionDivisor;
			obj.pt2.y *= (float)resolutionDivisor;
			LOG<<"("<<obj.pt1.x<<","<<obj.pt2.y<<")";

			obj.pt1.x -= robotWidth;
			obj.pt1.y += robotWidth;
			obj.pt2.x += robotWidth;
			obj.pt2.y -= robotWidth;

			if(useRadians)
			{
				obj.value /= (float)(M_PI / 180);
			}

			obj.type = OBJECT_TYPE_ROBOT;
			obj.layer = getNextLayer();

			LOG<<"RobotCorners = ("<<obj.pt1.x<<","<<obj.pt1.y<<")->("<<obj.pt2.x<<","<<obj.pt2.y<<")";
			LOG<<"Robot Layer = "<<obj.layer;

			setObject(obj);
		}

		objects.clear();
	}

	//TO DO: Go through all the boxes, fix their sizes and rotate them
	
	return true;
}

bool MapManager::saveStage(char* filePath, StageWorldOptions& options, float occupancyThreshold)
{
	if(!hasMap() || filePath == 0 || strlen(filePath) <= 0)
		return false;

//	resetView();

	//if the grid map is not currently visible, all the vectors that have been added to the map
	//may not have been added to the grid map, so make sure they are.
	if(!getViewGridMap())
	{
		pushAllVectorsOntoGrid();
	}
	StageWorldFileWriter writer;

	StageWorldOptions opt;	
	writer.setResolution((float)_resolution);	
	List<LineXYLayer> robots;
	robots.setModeQueue();

	_listObjects.resetIterator();

	LineXYLayer obj;

	while(_listObjects.readNext(obj))
	{
		if(obj.type == OBJECT_TYPE_ROBOT)
		{
			obj.pt1.x = (obj.pt1.x/_resolution - _mapMinX);
			obj.pt2.x = (obj.pt2.x/_resolution - _mapMinX);
			obj.pt1.y = (obj.pt1.y/_resolution - _mapMinY -1);
			obj.pt2.y = (obj.pt2.y/_resolution - _mapMinY-1);
			robots.push(obj);
		}
	}

	bool retval = false;	

	if(robots.getListSize() > 0)
	{
		writer.setRobots(&robots);

		opt = options;

		writer.setUseLaser(opt.hasLaser);
		writer.setUseSonar(opt.hasSonar);
		writer.setInitialPort(opt.initialPort);
	}
	else
	{
		writer.setRobots(0);
	}

	char baseFileName[512] = {0};
	char pnmFileName[512] = {0};
	char path[512] = {0};

	SosUtil::getBaseFileName(filePath,baseFileName);
	SosUtil::getPath(filePath,path);

	strcpy(pnmFileName,path);
	strcat(pnmFileName,"/");
	strcat(pnmFileName,baseFileName);
	strcat(pnmFileName,".pnm");

	bool retval1 = writer.save(filePath,pnmFileName);

	if(!retval1)
	{
		setErrorStrings(0,ERROR_FILE_SAVE(filePath));
		return false;
	}	

	unsigned char black = (unsigned char)0, white = (unsigned char)255;
	float gridVal = 0;

	float threshold = 0;
	
	long minX = 0, maxX = 0, minY = 0, maxY = 0;
	_gridLayer.getDimensions(minX,maxY,maxX,minY);

	long x = 0, y = 0;
	FILE* file = fopen(pnmFileName,"w");

	if(file == 0)
	{
		setErrorStrings(STD_ERROR_TITLE,ERROR_FILE_SAVE(pnmFileName));
		return false;
	}
	_setmode(_fileno(file),_O_BINARY);
	
	long width =  _mapMaxX - _mapMinX + 1;
	long height = _mapMaxY - _mapMinY + 1;	

	fprintf(file, "P5 \n# CREATOR: MapViewer by Shane O'Sullivan, http://mapviewer.skynet.ie\n%d %d\n255\n", width, height);

	for(y = _mapMaxY; y>=_mapMinY; y--)
	{
		for(x = _mapMinX; x <= _mapMaxX; x++)
		{
			gridVal = _gridLayer.read(x,y);

			//if the grid cell has the value -1, and it is OUTSIDE the main map, that is 
			//because there is a robot outside the main map.  In this case, treat the cell as if 
			//it is unoccupied, or white.  If the cell is -1 and INSIDE the main map, then 
			//treat it as unknown, and therefore black.
			if(gridVal == -1)
			{
				if(!SosUtil::between(x,minX,maxX) || !SosUtil::between(y,minY,maxY))
				{
					gridVal = 0;
				}
				else
				{
					gridVal = 1;
				}
			}

			if(gridVal < threshold)
			{
				fwrite(&black,sizeof(unsigned char),1,file);
			}
			else
			{
				fwrite(&white,sizeof(unsigned char),1,file);
			}			
		}
	}
	fclose(file);

	return true;
}



bool MapManager::saveMapViewerMap(char* filePath, bool saveGrid, bool saveVector)
{
	if(filePath == 0)
		return false;

	LOG<<"In MapManager::saveMapViewerMap("<<filePath<<","<<saveGrid<<","<<saveVector<<")";
	ofstream out;
	out.open(filePath);
	if(!out.is_open())
		return false;

	//only go to the trouble of removing the objects from the grid layers if
	//we want to save both the grid and the vectors, and there are actually vectors to remove
	bool removeObjects = (!(_listObjects.getListSize() == 0)) && saveGrid && saveVector;

	char buffer[256] = {0};

	LineXYLayer obj;
	List<LineXYLayer> tempObjList;
	tempObjList.setModeOrderedAsc();

	//remove the objects from the grid layer
	if(removeObjects)
	{
		_listObjects.resetIterator();
		while(_listObjects.readNext(obj))
		{
			_gridLayer.popObject(obj, _resolution);
			tempObjList.push(obj);
		}

		_gridLayer.deleteAllLayerInfo();
	}

	long west = 0, east = 0, south = 0, north = 0;

	
	out<<"resolution "<<_resolution<<endl;

	//state that the vectors are in mm, not related to the resolution
	//earlier versions of Map Viewer files do not have this
	out<<"resolution_off true"<<endl;  

	LOG<<"saveGrid = "<<saveGrid;

	if(saveGrid)
	{
		_gridLayer.getDimensions(west,north,east,south);

		LOG<<"SAVE:(20,787) = "<<_gridLayer.read(20,787);

		out<<"gridmap "<<endl;
		out<<"west "<<west<<" \neast "<<east<<" \nnorth "<<north<<" \nsouth "<<south<<endl; 
		out<<"data"<<endl;

		float val = 0, lookaheadVal = 0, lookaheadY = 0;	
		char* buf2 = 0;
		int pos1=0, pos2=0;
		
		for(long x = west; x <= east; x++)
		{
			for(long y = south; y <= north; y++)
			{
				val = _gridLayer.read(x,y);

				if(x == 35 && y == 780)
				{
					LOG;
				}
				
				if(val == 1.0f)
				{
					strcpy(buffer,"1");
				}
				else if(val < 0)
				{
					strcpy(buffer,"-1");
				}
				else if(val == 0.0f)
				{
					strcpy(buffer,"0");
				}
				else
				{
					SosUtil::floatToString(val,3,buffer);
					//buffer[0]='.';
					//buf2=_ecvt(val,3,&pos1,&pos2);
					//strcpy((buffer+1),buf2);
				}

			//	SosUtil::floatToString(val,3,buffer);
				
				lookaheadY = y+1;
				lookaheadVal = _gridLayer.read(x,lookaheadY);

				while(lookaheadY <= north && lookaheadVal == val)
				{
					lookaheadY++;
					lookaheadVal = _gridLayer.read(x,lookaheadY);
				}

				if(lookaheadY > y+1)
				{			
					out<<"["<<lookaheadY - y <<' ';
					y = (long)(lookaheadY - 1);			
				}

				if(x == 20 && y == 787)
				{
					LOG<<"(20,787) val = "<<val<<", string val = "<<buffer<<", double val = "<<(double)val;
				}
				
				if(buffer[0] == '0' && buffer[1] == '.')
				{
					out<<buffer+1; 
				}
				else
				{
					out<<buffer;
				}
				out<<' ';
							
			}
			out<<'\n';
		}
	}

	//if we have vectors to save then save them
	if(saveVector && _listObjects.getListSize() != 0)
	{
		out<<"\nvectorobjects"<<endl;
		_listObjects.resetIterator();
		while(_listObjects.readNext(obj))
		{
			switch(obj.type)
			{
			case OBJECT_TYPE_LINE:
				out<<"line ";
				break;
			case OBJECT_TYPE_RECTANGLE:
				out<<"rect ";
				break;
			case OBJECT_TYPE_RECTANGLE_FILLED:
				out<<"rectfill ";
				break;
			case OBJECT_TYPE_ROBOT:
				out<<"robot ";
				break;
			}
			out<<obj.layer<<" "<<obj.value<<" "<<obj.pt1.x<<" "<<obj.pt1.y<<" "<<obj.pt2.x<<" "<<obj.pt2.y<<endl;
		}

		//if we removed the objects from the grid layers, add them back
		if(removeObjects)
		{
			//add the vectors back to the grid layer
			while(tempObjList.popHead(obj))
			{
				_gridLayer.pushObject(obj,_resolution);
			}
		}
	}
	
//	resetView();

	return true;
}
	
bool MapManager::loadMapViewerMap(char* filePath)
{
	if(filePath == 0)
	{
		setErrorStrings("Error","File path is null");
		return false;
	}

	LOG<<"In MapManager::loadMapViewerMap("<<filePath<<")";

	MapViewerFileParser parser;
	float west = 0,east = 0,north = 0,south =0;	

	//if we fail to parse the map, return false
	if(!parser.parseMapViewerMap(filePath))
	{
		setErrorStrings("Error","Failed to parse the file ", filePath);
		
		return false;
	}

	bool loadGrid = false, loadVectors = false;	

	resetAllObjects();
	LineXYLayer obj;

	_resolution = parser.getResolution();

	parser.getDimensions(west,north,east,south);

	GridMap<float>* loadedMap = 0;

	loadGrid = parser.hasGridMap();

	LOG<<"LoadGrid = "<<loadGrid;

	if(loadGrid)
	{
		loadedMap = new GridMap<float>(1000,0,0);

		if(parser.getGridMap(loadedMap))
		{
			LOG<<"LOAD:(20,787) = "<<loadedMap->getGridRef(20,787);
			addMap(loadedMap,true);//performs shallow copy, destroys this copy
			_viewGridMap = true;
		}
		else
		{
			loadGrid = false;//loading the grid failed, create a new one later
		}		

		setViewGridMap(true);
		delete loadedMap;	
		loadedMap = 0;
	}
	else
	{
		newMap((long)west,(long)east,(long)south,(long)north);
		setViewGridMap(false);
	}

	loadVectors = parser.hasVectors();

	bool multiplyRes = !parser.vectorsInMM();

	if(loadVectors)
	{
		parser.resetObjectIterator();

		while(parser.getNext(obj))
		{
			if(loadGrid)
			{
				obj.layer = getNextLayer();
			}
			else
			{
				obj.layer = getNextNegativeLayer();
			}

			if(multiplyRes)
			{	
				obj.pt1.x *= _resolution;		
				obj.pt1.y *= _resolution;
				obj.pt2.y *= _resolution;
				obj.pt2.x *= _resolution;
			}

			setObject(obj);
		}
	}

	setViewVectorMap(loadVectors);
	setViewGridMap(loadGrid);

	LineXYLong temp = _vectorBoundary;

	SosUtil::ensureSmaller(temp.pt1.x,temp.pt2.x);
	SosUtil::ensureSmaller(temp.pt1.y,temp.pt2.y);

	_vectorBoundary.pt1.x = temp.pt1.x;
	_vectorBoundary.pt2.x = temp.pt2.x ;
	_vectorBoundary.pt1.y = temp.pt1.y;
	_vectorBoundary.pt2.y = temp.pt2.y;

//	resetView();

	LOG<<"At end of MapManager::loadMapViewerMap("<<filePath<<")";

	return true;
}

bool MapManager::saveRossum(char* filePath)
{
	if(filePath == 0 || _listObjects.getListSize() == 0)
	{
		return false;
	}

	const float LINE_WIDTH = 0.05f;

	ofstream out(filePath,ios::out);
	//out.open(filePath);

	if(!out.is_open())
	{
		return false;
	}

	//output caption on top
	out<<"/*\n"<<stripPath(filePath)<<" created by MapViewer, http://mapviewer.skynet.ie \n*/";

	//units are always in metres - only muppets use anything else
	out<<"\nunits: meters;"<<endl;

	out<<"caption:\""<<SosUtil::stripPath(filePath)<<"\";";

	LineXYLayer obj;
	_listObjects.resetIterator();
	long wallCount = 0;
	float mid = 0, minX=0, maxX=0, minY=0, maxY=0;
	float pt1x = 0, pt1y = 0, pt2x = 0, pt2y = 0;
	float midX = 0, midY = 0;
	double area = 0;
	char buffers[4][256];

	for(int i = 0; i< 4; i++)
	{
		for(int j = 0; j<256; j++)
		{
			buffers[i][j] = '\0';
		}
	}

	long numRobots = 0;

	while(_listObjects.readNext(obj))
	{
		pt1x = (obj.pt1.x)/1000;
		pt1y = (obj.pt1.y)/1000;
		pt2x = (obj.pt2.x)/1000;
		pt2y = (obj.pt2.y)/1000;

		area = sqrt((pt1x -pt2x)*(pt1x-pt2x) + (pt1y -pt2y)*(pt1y-pt2y));
		//if the object is a single point, don't save it, otherwise it'll cause a parse exception
		if(area < 0.0001 )
		{
			continue;
		}
		
		switch(obj.type)
		{
		case OBJECT_TYPE_LINE:
		
			out<<"\nwall "<<wallCount++<<"{ geometry:\t"
				<<(double)pt1x<<",\t"
				<<(double)pt1y<<",\t"
				<<(double)pt2x<<",\t"
				<<(double)pt2y<<",\t"
				<<"0.05;}";
			
			break;
		case OBJECT_TYPE_RECTANGLE:
			
			//output this as four lines, each of width 0.05
			minX = pt1x;
			maxX = pt2x;
			minY = pt1y;
			maxY = pt2y;
			SosUtil::ensureSmaller(minX,maxX);
			SosUtil::ensureSmaller(minY,maxY);

			if((maxX - minX) > LINE_WIDTH &&(maxY - minY)> LINE_WIDTH)
			{
		//		LOG<<"OBJECT_TYPE_RECTANGLE 1";
				out<<"\nwall "<<wallCount++<<"{ geometry:\t";//top line
				out<<minX<<",\t"
					<<maxY - LINE_WIDTH/2<<",\t"
					<<maxX<<",\t"
					<<maxY - LINE_WIDTH/2<<",\t"
					<<LINE_WIDTH<<";}";

				out <<"\nwall "<<wallCount++<<"{ geometry:\t";//left line
				out <<minX + LINE_WIDTH/2<<",\t"
					<<maxY<<",\t"
					<<minX + LINE_WIDTH/2<<",\t"
					<<minY<<",\t"
					<<LINE_WIDTH<<";}";

				out<<"\nwall "<<wallCount++<<"{ geometry:\t";//right line
				out	<<maxX - LINE_WIDTH/2<<",\t"
					<<maxY<<",\t"
					<<maxX - LINE_WIDTH/2<<",\t"
					<<minY<<",\t"
					<<LINE_WIDTH<<";}";

				out<<"\nwall "<<wallCount++<<"{ geometry:\t";//bottom line
				out<<minX<<",\t"
					<<minY + LINE_WIDTH/2<<",\t"
					<<maxX<<",\t"
					<<minY + LINE_WIDTH/2<<",\t"
					<<LINE_WIDTH<<";}";
			}
			else if((maxX - minX) < (maxY - minY))
			{
		//		LOG<<"OBJECT_TYPE_RECTANGLE 2";
				mid = SosUtil::midWay(minX,maxX);
				out<<"\nwall "<<wallCount++<<"{ geometry:\t";
				out<<mid<<",\t"
					<<maxY<<",\t"
					<<mid<<",\t"
					<<minY<<",\t"
					<<maxX - minX<<";}";

			}
			else
			{
		//		LOG<<"OBJECT_TYPE_RECTANGLE 3";
				mid = SosUtil::midWay(minY,maxY);
				out<<"\nwall "<<wallCount++<<"{ geometry:\t";
				out	<<maxX<<",\t"
					<<mid<<",\t"					
					<<minX<<",\t"
					<<mid<<",\t"
					<<maxY - minY<<";}";
			}

			break;
		case OBJECT_TYPE_RECTANGLE_FILLED:

			//output this as a vertical line that goes down the middle of the rectangle.
			//The line is as wide as the rectangle
			mid = (SosUtil::midWay(obj.pt1.x,obj.pt2.x))/1000;
			out<<"\nwall "<<wallCount++<<"{ geometry:\t";
			out<<mid<<",\t"
				<<(obj.pt1.y)/1000<<",\t"
				<<mid<<",\t"
				<<(obj.pt2.y)/1000<<",\t"
				<<(SosUtil::fabs(obj.pt1.x -obj.pt2.x))/1000<<";}";

			break;
		case OBJECT_TYPE_ROBOT:
			//get the centre of the robot
			midX = (SosUtil::midWay(obj.pt1.x,obj.pt2.x))/1000;
			midY = (SosUtil::midWay(obj.pt1.y,obj.pt2.y))/1000;

			out<<"\nplacement home";
			
			if(numRobots > 0)
			{
				out<<numRobots;
			}

			out<<" {\n\tlabel:\t\t \"Home";

			if(numRobots > 0)
			{
				out<<numRobots;
			}

			out<<"\";\n\tgeometry:\t"<<midX<<", "<<midY<<", "<<obj.value<<", "<<((float)ROBOT_RADIUS)/(float)1000<<";";
			out<<"\n\tlineColor:\tgreen;\n\tlineWidth:\t3;\n}";

			numRobots++;
			break;

		}
	}

	out.close();
	return true;
}


bool MapManager::loadPath(char* filePath)
{
	if(filePath == 0)
	{
		LOG<<"filePath is NULL";
		return false;
	}

	PointXYLong pt(0,0);
	List<PointXYLong>* ptList = 0;

	ifstream in;
	in.open(filePath);

	if(!in.is_open())
	{
		LOG<<"Could not open file "<<filePath;
		return false;
	}

	char buffer1[250]={0}, buffer2[250]={0};
	bool parsedCorrectly = true;

	in>>buffer1;

	//the first entry in a path file must be the word "path"
	if(strcmp(buffer1,"path") != 0)
	{
		in.close();
		LOG<<"File doesn't begin with 'path'";
		return false;
	}

	List<List<PointXYLong>*> tempListofLists;

	in>>buffer1;
	while(!in.eof())
	{
		//each list of path points must start with the word "begin"
		if(strcmp(buffer1,"begin") != 0)
		{
			parsedCorrectly = false;
			LOG<<"First entry should be 'begin', but it is "<<buffer1;
			break;
		}

		ptList = new List<PointXYLong>;
		
		tempListofLists.push(ptList);

		in>>buffer1;

		while(!in.eof())
		{
			if(!SosUtil::is_numeric(buffer1))
			{
				//the only non-numeric entry allowed after "begin" is "end"
				if(strcmp(buffer1,"end") != 0)
				{
					parsedCorrectly = false;
					LOG<<"1.Token should be 'end', but it is "<<buffer1;
				}
				break;				
			}

			in>>buffer2;

			if(in.eof())
				break;

			if(!SosUtil::is_numeric(buffer2))
			{
				//the only non-numeric entry allowed after "begin" is "end"
				if(strcmp(buffer2,"end") != 0)
				{
					parsedCorrectly = false;
					LOG<<"2.Token should be 'end', but it is "<<buffer1;
				}
				break;				
			}

			pt.x = atoi(buffer1);
			pt.y = atoi(buffer2);
			ptList->push(pt);

			in>>buffer1;
		}

		if(!parsedCorrectly)
		{
			LOG<<"Not parsed properly - getting out of main loop";
			break;
		}	

		in>>buffer1;
	}

	//if the file wasn't parsed properly, then clean up the lists of points, and return false
	if(!parsedCorrectly)
	{
		while(tempListofLists.popHead(ptList))
		{
			if(ptList != 0)
			{
				ptList->clear();
				delete ptList;
			}
		}
		return false;
	}

	LineXYLong line(0,0,0,0);
	PointXYLong pt1, pt2;
	tempListofLists.resetIterator();

	while(tempListofLists.readNext(ptList))
	{
		if(ptList != 0)
		{
			_pathLists.push(ptList);//store the path point list permanently
			
			ptList->resetIterator();
			if(ptList->readNext(pt1)) //get the first point in the path
			{					
				while(ptList->readNext(pt2))
				{					
					line.pt1 = pt1;
					line.pt2 = pt2;
					_listPathLines.push(line);
				
					pt1 = pt2;
				}			
			}
		}
	}

	_viewPath = true;
	return true;
}

//the format of a path file is as follows
//path
//begin
//x y 
//x y 
// ......
//end
//begin
// ...
//end
// .... etc  where each begin-end pair delimit a single path, as there can be multiple paths in one file
// 
bool MapManager::savePath(char* filePath)
{
	if(filePath == 0 || _listPathLines.getListSize() == 0)
		return false;

	ofstream out;
	out.open(filePath);

	if(!out.is_open())
		return false;
	
	List<PointXYLong>* list = 0;
	PointXYLong pt;

	_pathLists.resetIterator();

	out<<"path";
	while(_pathLists.readNext(list))
	{
		if(list == 0 || list->getListSize() == 0)
		{
			continue;//if there's nothing in the list, move on to the next one
		}
		
		list->resetIterator();
		out<<"begin"<<endl;
		while(list->readNext(pt))
		{
			out<<pt.x<<" "<<pt.y<<endl;
		}
		out<<"end"<<endl;
	}

	out.close();
	return true;
}

void MapManager::setObject(LineXYLayer& object)
{
	_listObjects.push(object);

//	LOG<<"setObject() set: "<<object;

	if(_listObjects.getListSize() > 1)
	{
		_vectorBoundary.pt1.x = SosUtil::minVal(_vectorBoundary.pt1.x,(long)object.pt1.x);
		_vectorBoundary.pt1.x = SosUtil::minVal(_vectorBoundary.pt1.x,(long)object.pt2.x);
		_vectorBoundary.pt2.x = SosUtil::maxVal(_vectorBoundary.pt2.x,(long)object.pt1.x);
		_vectorBoundary.pt2.x = SosUtil::maxVal(_vectorBoundary.pt2.x,(long)object.pt2.x);

		_vectorBoundary.pt1.y = SosUtil::minVal(_vectorBoundary.pt1.y,(long)object.pt1.y);
		_vectorBoundary.pt1.y = SosUtil::minVal(_vectorBoundary.pt1.y,(long)object.pt2.y);
		_vectorBoundary.pt2.y = SosUtil::maxVal(_vectorBoundary.pt2.y,(long)object.pt1.y);
		_vectorBoundary.pt2.y = SosUtil::maxVal(_vectorBoundary.pt2.y,(long)object.pt2.y);
	}
	else
	{
		_vectorBoundary.pt1.x = (long)SosUtil::minVal(object.pt1.x,object.pt2.x);
		_vectorBoundary.pt2.x = (long)SosUtil::maxVal(object.pt1.x,object.pt2.x);
		_vectorBoundary.pt1.y = (long)SosUtil::minVal(object.pt1.y,object.pt2.y);
		_vectorBoundary.pt2.y = (long)SosUtil::maxVal(object.pt1.y,object.pt2.y);
	}

	//if the grid map is visible, then the layer of the object will be positive
	//and it should be added to the grid map.  Otherwise, do not add it.
	if(object.layer > 0)
	{
		_gridLayer.pushObject(object,_resolution);
	}
}



bool MapManager::redo()
{
	int lastUndoAction = 0;
	LineXYLayer object, tempObj;
	LayerValue<LineXYLayer> objectUndone;
	int type = 0;
	long layer = 0;
	LineXYLong retval;
	LayerValue<LineXYLayer> objectReplaced;

	LineXYLong currentMapSize = _gridLayer.getDimensions();

	if(!_listUndoActions.popHead(lastUndoAction))
	{
		return false;//return false since we did nothing
	}

	LOG<<"redo() got action "<<lastUndoAction;
	switch(lastUndoAction)
	{
	case ACTION_UNDO_SET_POINT:
		if(_listPointsUndone.popHead(layer))
		{
			_listPoints.push(layer);
			retval = _gridLayer.redoLayer(layer);
//			addChangedArea(retval.pt1.x,retval.pt1.y,retval.pt2.x,retval.pt2.y,currentMapSize);
			addUserAction(ACTION_SET_POINT, false);//set 2nd param to false so undo artifacts are not deleted
			return true;
		}
		
		break;
	case ACTION_UNDO_SET_OBJECT:
		if(_listObjectsUndone.popHead(object))
		{
			setObject(object);
			addUserAction(ACTION_SET_OBJECT, false);
//			addChangedArea(object.pt1.x,object.pt1.y,object.pt2.x,object.pt2.y,currentMapSize);

			return true;
		}
		else
		{
			return false;
		}

		break;
	case ACTION_UNDO_DELETE_POINT:
		if(_listPointsUndone.popHead(layer))
		{
			_gridLayer.redoLayer(layer);
			addUserAction(ACTION_SET_POINT);
			_listPoints.push(layer);
		}

		break;
	case ACTION_UNDO_DELETE_OBJECT:

		break;
	case ACTION_UNDO_REPLACE_OBJECT:
		//Undoing a replace operation is the same as deleting an object & placing it on the _listObjectsUndone list
		//then setting an object and putting it on the _listObjects list
		//So, pop the head of the _listObjectsUndone list, pop the latest pushed object, put the first onto the 
		//_listObjects list, and the second onto the _listObjectsReplaced.  Then push the user action REPLACE
		if(_listObjectsReplacedUndone.popHead(objectUndone))
		{
			tempObj.layer = objectUndone.layerNumber;
			if(_listObjects.popVal(tempObj,object))
			{
				if(object.layer > 0)
				{
					_gridLayer.popObject(object,_resolution);
				}
				
				objectReplaced.layerNumber = objectUndone.value.layer;
				objectReplaced.value = object;
				_listObjectsReplaced.push(objectReplaced);
				setObject(objectUndone.value);
				addUserAction(ACTION_REPLACE_OBJECT, false);
		//		addChangedArea(objectUndone.value.pt1.x,objectUndone.value.pt1.y,
		//						objectUndone.value.pt2.x,objectUndone.value.pt2.y,currentMapSize);
		//		addChangedArea(objectReplaced.value.pt1.x,objectReplaced.value.pt1.y,objectReplaced.value.pt2.x,objectReplaced.value.pt2.y,currentMapSize);
		
				return true;
			}
		}
		
		break;
	default:

		break;
	}

	return false;
}

bool MapManager::undo()
{	
	long minLayer = 0, maxLayer = 0;
	const int ansLine = 0, ansRect = 1, ansRectFilled = 2;
	int whichObject = -1;

	LineXYLayer val, val2;
	List<LineXYLayer>* list = 0;
	bool ignorePoints = false;

	LineXYLong ret(0,0,0,0);
	LineXYLong currentMapSize = _gridLayer.getDimensions();
	LayerValue<LineXYLayer> replacedObject;
	
	int lastAction = 0;
	long layer = 0;

	if(!_listUserActions.popHead(lastAction))
	{
		return false;
	}

	switch(lastAction)
	{
	case ACTION_SET_POINT://treat setting a single point and filling an area as the same thing
		
		//get last layer of points drawn and delete that layer from the gridLayer
		if(_listPoints.popHead(layer))
		{
			_listPointsUndone.push(layer);
			ret = _gridLayer.deleteLayer(layer);
//			addChangedArea(ret.pt1.x,ret.pt1.y,ret.pt2.x,ret.pt2.y,currentMapSize);
			addUndoAction(ACTION_UNDO_SET_POINT);
			return true;
		}
		return false;
		

	case ACTION_SET_OBJECT:
		
		if(!_listObjects.popHead(val))
		{
			return false;
		}

//		addChangedArea(val.pt1.x,val.pt1.y,val.pt2.x,val.pt2.y,currentMapSize);

		//if the line has been added to the map, it will have a positive layer number
		//so remove it from the map
		if(val.layer >= 0)
		{
			_gridLayer.popObject(val,_resolution);
		}

		_listObjectsUndone.push(val);
		
		addUndoAction(ACTION_UNDO_SET_OBJECT);	
		return true;

		break;
	
	case ACTION_DELETE_OBJECT:
		
		//get the object that was deleted
		if(!_listObjectsDeleted.popHead(val))
		{
			return false;
		}

		setObject(val);

		addUndoAction(ACTION_UNDO_DELETE_OBJECT);
		return true;
		break;
	
	case ACTION_REPLACE_OBJECT:
		
		if(!_listObjectsReplaced.popHead(replacedObject))
		{
			
			return false;
		}

		val.layer = replacedObject.layerNumber;		

		//get the object that replaced the deleted 
		if(!_listObjects.popVal(val,val2))
		{
			return false;
		}

		if(val2.layer > 0)
		{
			_gridLayer.popObject(val2,_resolution);
		}
//		addChangedArea(val2.pt1.x,val2.pt1.y,val2.pt2.x,val2.pt2.y,currentMapSize);
		
		setObject(replacedObject.value); //re-set the original object

		//reuse this object just to push it onto the list
		replacedObject.layerNumber = replacedObject.value.layer;//add the layer of the object that replaced this one
		replacedObject.value = val2; //add the object that was replaced

		_listObjectsReplacedUndone.push(replacedObject);

//		addChangedArea(val.pt1.x,val.pt1.y,val.pt2.x,val.pt2.y,currentMapSize);

		addUndoAction(ACTION_UNDO_REPLACE_OBJECT);

		return true;
		break;
	
	}
	
	return false;//didn't undo anything
}

void MapManager::removeOldestUndoAction()
{
	long val = 0;
	LineXYLayer line;
	int action;

	if(!_listUndoActions.popTail(action))
		return;
	
	switch(action)
	{
	case ACTION_UNDO_SET_POINT://if we're removing an undo place for points, might as well delete the layer
		//storing the list of points in that layer
		if(_listPointsUndone.popTail(val))
		{
			_gridLayer.deleteLayerPermanently(val);
		}		
		break;
	case ACTION_UNDO_SET_OBJECT:
		_listObjectsUndone.popTail(line);		
		break;
	
	case ACTION_UNDO_DELETE_OBJECT:
		_listObjectsDeleted.popTail(line);
		break;		
	}
}

void MapManager::addUndoAction(int action)
{
	_listUndoActions.push(action);

	if(_listUndoActions.getListSize() > NUM_UNDO_STEPS)
	{
		removeOldestUndoAction();
	}
}

long MapManager::getNextLayer()
{
	_pointsAddedToUserActions = false;
	return ++_latestLayer;
	
}

long MapManager::getNextNegativeLayer()
{
	_gotNegLayer = true;
	return --_latestNegLayer;
}
void MapManager::addUserAction(int action, bool clearUndoneArtifacts)
{
	int temp = 0;
	long layer = 0;
	
	//only allow a certain number of undo steps to preserve memory
	if(_listUserActions.getListSize() > NUM_UNDO_STEPS)
	{
		
		_listUserActions.popTail(temp); 
		
		switch(temp)
		{
		case ACTION_SET_POINT:		

			//if we're forgetting that a point was added, remove the list of points in the grid
			//that remember it - leave the effect on the grid though
			if(_listPoints.popTail(layer))
			{
				_gridLayer.deleteLayerPermanently(layer);
			}

			break;
		}
	}

	//if the user has performed some undo steps, and then does something else, then empty
	//the lists that store the undone objects
	if(!IS_UNDO_ACTION(action) && clearUndoneArtifacts)
	{		
		_listObjectsUndone.clear();
		_listObjectsReplacedUndone.clear();
		
		while(_listPointsUndone.popHead(layer))
		{
			_gridLayer.deleteLayerPermanently(layer);
		}
	}

	_listUserActions.push(action);
}

//These X and Y values are in grid coordinates, not in MM!
void MapManager::setPoint(long x, long y, float value)
{
	if(!hasMap())
		return;
	
	_gridLayer.push(x,y,_latestLayer,value);

	if(!_pointsAddedToUserActions)
	{
		_pointsAddedToUserActions = true;
		addUserAction(ACTION_SET_POINT);
		_listPoints.push(_latestLayer);
	}	
}

LineXYLayer MapManager::setRobot(long centreX, long centreY)
{
	LineXYLayer robot;
	if(!hasMap())
		return robot;

	LOG<<"setRobot("<<centreX<<","<<centreY<<")";
	
	robot.layer = getNextLayer();
	robot.pt1.x = centreX + ROBOT_RADIUS;
	robot.pt2.x = centreX - ROBOT_RADIUS;
	robot.pt1.y = centreY + ROBOT_RADIUS;
	robot.pt2.y = centreY - ROBOT_RADIUS;

	robot.value = 0;
	robot.type = OBJECT_TYPE_ROBOT;

	setObject(robot);	
	addUserAction(ACTION_SET_OBJECT);

	return robot;
}

void MapManager::setLine(long x1, long y1, long x2, long y2, float value)
{	
	LOGENTRY("setLine")
	if(!hasMap())
		return;

	LOG<<"setLine got mm("<<x1<<","<<y1<<") -> ("<<x2<<","<<y2<<")";

	long layerVal = -1;	

	LOG<<"setLine(): Just before pushing line onto the grid layer";

	//only modify the gridmap if we are viewing it right now.  otherwise, leave the 
	//line with a layer value of -1
	if(_viewGridMap)
	{
		layerVal = getNextLayer();
	}
	else
	{
		layerVal = getNextNegativeLayer();
	}

	LineXYLayer line;
	line.pt1.x = x1;
	line.pt1.y = y1;
	line.pt2.x = x2;
	line.pt2.y = y2;
	
	line.layer = layerVal;
	line.value = value;
	line.type = OBJECT_TYPE_LINE;
	setObject(line);

	addUserAction(ACTION_SET_OBJECT);

	LOGEXIT("setLine")	
}

LineXYLayer	MapManager::fixRobotSize(LineXYLayer object)
{
	if(object.type != OBJECT_TYPE_ROBOT)
	{
		return object;
	}

	PointXY centrePt(0,0);

	object.pt1.x = SosUtil::midWay(object.pt1.x,object.pt2.x);
	object.pt1.y = SosUtil::midWay(object.pt1.y,object.pt2.y);
	
	object.pt2 = object.pt1;

	object.pt1.x -= ROBOT_RADIUS;
	object.pt1.y += ROBOT_RADIUS;
	object.pt2.x += ROBOT_RADIUS;
	object.pt2.y -= ROBOT_RADIUS;

	return object;
}

//These X and Y values are in grid coordinates, not in MM!
float MapManager::getPointVal(long x, long y)
{	
	if(!hasMap())
	{
		return -1;
	}

	return _gridLayer.read(x,y);
}


//fills in a rectangle with the specified value based on two coordinates
//in the window view
void MapManager::setRectangle(long x1, long y1, long x2, long y2, float value)
{	
	LOGENTRY("setRectangle")
	if(!hasMap())
		return;

	long layerVal= 0;

	SosUtil::ensureSmaller(x1,x2);
	SosUtil::ensureSmaller(y1,y2);
	
	//only modify the gridmap if we are viewing it right now.  otherwise, leave the 
	//line with a layer value of -1
	if(_viewGridMap)
	{
		layerVal = getNextLayer();
	}
	else
	{
		layerVal = getNextNegativeLayer();
	}

	LineXYLayer object(layerVal,value,OBJECT_TYPE_RECTANGLE,x1,y1,x2,y2);
	setObject(object);

	addUserAction(ACTION_SET_OBJECT);

	LOGEXIT("setRectangle")
	
}

//fills in a rectangle with the specified value based on two coordinates
//in the window view
void MapManager::setRectangleFilled(long x1, long y1, long x2, long y2, float value)
{
	LOGENTRY("setRectangleFilled")
	if(!hasMap())
		return;

	long layerVal = 0;

	LineXYLong currentMapSize = _gridLayer.getDimensions();

	SosUtil::ensureSmaller(x1,x2);
	SosUtil::ensureSmaller(y1,y2);
	
	//only modify the gridmap if we are viewing it right now.  otherwise, leave the 
	//line with a layer value of -1
	if(_viewGridMap)
	{
		layerVal = getNextLayer();	
	}
	else
	{
		layerVal = getNextNegativeLayer();
	}

	LineXYLayer object(layerVal,value,OBJECT_TYPE_RECTANGLE_FILLED, x1,y1,x2,y2);
	setObject(object);
	
	addUserAction(ACTION_SET_OBJECT);
	
	LOGEXIT("setRectangleFilled")
	
}

void MapManager::pushAllVectorsOntoGrid()
{
	LOG<<"in pushAllVectorsOntoGrid()";
	LineXYLayer line;
	ListUnordered<LineXYLayer> tempList;
	//tempList.setModeStack();
	_listObjects.resetIterator();

	while(_listObjects.readNext(line))
	{
		if(line.layer < 0)
		{
			tempList.push(line);
			_listObjects.remove();//remove the object currently being accessed
		}
	}
	
	LOG<<"Got "<<tempList.getListSize()<<" objects to push";

	while(tempList.popTail(line))
	{
		LOG<<"Popped the object "<<line;
		line.layer = getNextLayer();
		
		_gridLayer.pushObject(line,_resolution);

		LOG<<"pushAllVectorsOntoGrid(): Pushed "<<line<<" onto grid";
		
		_listObjects.push(line);
	}
}


void MapManager::enableLayerInfo()
{
	_gridLayer.enableLayerInfo();
}
void MapManager::disableLayerInfo()
{
	_gridLayer.disableLayerInfo();
}

void MapManager::setViewGridMap(bool viewGridMap)
{
	LOG<<"setViewGridMap("<<viewGridMap<<")";
	LineXYLayer line, tempLine;
	long x1 = 0, y1 = 0, x2 = 0, y2 = 0;
	long layerVal = 0;
	List<LineXYLayer> tempList;
	tempList.setModeOrderedDesc();

	//if the grid map was previously hidden, and now it's being shown,
	//then look for any objects that have not yet been added to the grid layers
	if(_viewGridMap == false && viewGridMap == true && _gotNegLayer)
	{
		_gotNegLayer = false;

		if(_myMap == 0)
		{
			LOG<<"setViewGridMap() Creating a new map";
			newMap(0,100,0,100);
		}
		pushAllVectorsOntoGrid();
		
	}

	_viewGridMap = viewGridMap;

}


void MapManager::fillArea(long xVal, long yVal,float valueToSet,double tolerance)
{
	LOGENTRY("fillArea")
	
	if(!hasMap())
	{
		return ;
	}

	float xValFloat = 0, yValFloat = 0;
	mmToGrid(xVal,yVal,xValFloat,yValFloat);

	xVal = (long)xValFloat;
	yVal = (long)yValFloat;

	LineXYLong retval;
	long layer = getNextLayer();

	GridMap<float>* markMap = 0;
	double origValue = 0;

	long east,west,north,south;
	
	List<PointXYLong> list;
	list.setModeStack();
	
	const int MASK_SIZE = 4;
	
	const int mask[MASK_SIZE][2] = { {0,1}, {-1,0}, {1,0}, {0,-1}};
			
	double gridVal = 0;
	
	origValue = _gridLayer.read(xVal,yVal);
	
	markMap = new GridMap<float>(50,1,0);
	
	PointXYLong tempPoint;
	tempPoint.x = xVal;
	tempPoint.y = yVal;

	list.push(tempPoint);
	
	long pushCount = 0,processCount = 0;
	
	//pt1 is the top left point, pt2 is the bottom right point
	retval.pt1.x = tempPoint.x;
	retval.pt2.x = tempPoint.x; 
	retval.pt1.y = tempPoint.y;
	retval.pt2.y = tempPoint.y;
	
	while(list.popHead(tempPoint))
	{
		processCount++;
		
		xVal = tempPoint.x;
		yVal = tempPoint.y; 
		_gridLayer.push(xVal,yVal,layer,valueToSet);
			
		//fill in the children first
		for(int i = 0; i< MASK_SIZE; i++)
		{	
			gridVal = _gridLayer.read(xVal + mask[i][0],yVal + mask[i][1]);

			east = _gridLayer.getDimensions(EAST);
			west = _gridLayer.getDimensions(WEST);
			north = _gridLayer.getDimensions(NORTH);
			south = _gridLayer.getDimensions(SOUTH);

			//gridVal = mapToUse->getGridRef(xVal + mask[i][0],yVal + mask[i][1]);
			if(	markMap->getGridRef(xVal + mask[i][0],yVal + mask[i][1]) != 1 &&
				gridVal <= origValue + tolerance && gridVal >= origValue - tolerance &&
				xVal + mask[i][0] <= east &&  xVal + mask[i][0] >= west &&
				yVal + mask[i][1] <= north && yVal + mask[i][1] >= south )
			{
				markMap->updateGridRef(1,xVal + mask[i][0],yVal + mask[i][1]);
				
				//stick all the children on the list that haven't already been added to the list
				tempPoint.x = xVal + mask[i][0];
				tempPoint.y = yVal + mask[i][1];
				list.push(tempPoint);
				pushCount++;
			}
		}  		
	}
	
	delete markMap;
	markMap = 0;
	origValue = 0;	

	_listUserActions.push(ACTION_SET_POINT);
	_listPoints.push(layer);

	LOGEXIT("fillArea")	
}


bool MapManager::convertGridToLineWithVoronoi(float minThreshold, float maxThreshold,
												  bool filterByCellValue, float valueToSet)
{
	if(!hasMap())
		return false;

	GridMap<float>tempMap(1000,1,0);

	SosUtil::ensureSmaller(minThreshold,maxThreshold);

	float minDist = 2.0f;

	if(!filterByCellValue)
	{
		minDist = 3;
	}

	bool resetIterator = false;

	const int above = 0, below = 1, right = 2, left = 3;

	LineXYLong dim = _gridLayer.getDimensions();
	SosUtil::ensureSmaller(dim.pt1.x,dim.pt2.x);
	SosUtil::ensureSmaller(dim.pt2.y,dim.pt1.y);

	long x = 0, y = 0;
	long north = dim.pt1.y, south = dim.pt2.y, east = dim.pt2.x, west = dim.pt1.x;

	//remove any vectors that are considered to be occupied, as they will be replaced
	//by the line-fitting algorithm
	LineXYLayer object;
	List<LineXYLayer> tempList;

	_listObjects.resetIterator();

	//for any objects whose values are in the threshold being processed, remove them
	while(_listObjects.readNext(object))
	{
		if(object.type != OBJECT_TYPE_ROBOT && SosUtil::between(object.value, minThreshold,maxThreshold))
			tempList.push(object);
	}

	while(tempList.popHead(object))
	{
		_listObjects.popVal(object,object);
	}

	//copy the map out of the layer object
	west = _gridLayer.getDimensions(WEST);
	east = _gridLayer.getDimensions(EAST);
	north = _gridLayer.getDimensions(NORTH);
	south= _gridLayer.getDimensions(SOUTH);

	float val = 0;
	bool ignoreCell = false;
	float * tempRow = new float[east - west +3];

	if(tempRow == 0)
	{
		setErrorStrings(STD_ERROR_TITLE,STD_NOT_ENOUGH_MEMORY);
		return false;
	}

	//any occupied cell that is surrounded on all sides by other occupied cells must be set to 0
	//This is so that the outline of the solid object is drawn instead of the centre of it
	if(filterByCellValue)
	{
		for(y = south; y<= north; y++)
		{
			_gridLayer.copyRow(tempRow,y,west,east);
			for(x = west; x<= east; x++)
			{
				val = tempRow[x - west];
				if(SosUtil::between(val,minThreshold, maxThreshold))
				{
					if(x == west || x == east || y == south || y == north)
					{
						tempMap.updateGridRef(1,x,y);
					}
					else
					{
						ignoreCell = true;
						for(long xNum = x - 1; xNum <= x+1; xNum++)
						{
							for(long yNum = y-1; yNum <= y+1; yNum++)
							{
								if(!SosUtil::between(_gridLayer.read(xNum,yNum),minThreshold,maxThreshold))
								{
									ignoreCell = false;
									yNum =y+2;
									xNum = x+2;//break out of both for loops
								}
							}
						}
						if(ignoreCell)
						{
							tempMap.updateGridRef(0,x,y);
						}
						else
						{
							tempMap.updateGridRef(1,x,y);
						}
					}
				}
				else
				{
					tempMap.updateGridRef(0,x,y);
				}
			}
		}
	}

	//if we are filtering by contrast, search around a cell to see if the difference between it and
	//a neighbour is >= the threshold.  If it is, and another neighbour has not already
	//been used, then set the value in tempMap as an obstacle (to 1)
	bool thresholdPassed = false;
	bool neighbourMarked = false;
	long markedX = 0,markedY = 0;
	float neighbourVal = 0, maxVal = 0;
	
	if(!filterByCellValue)
	{
		for(y = south;  y<= north; y++)
		{
			_gridLayer.copyRow(tempRow,y,west,east);
			for(x = west; x<= east; x++)
			{
				val = tempRow[x - west];
				thresholdPassed = neighbourMarked = false;
				maxVal = val;
				markedX = x;
				markedY = y;
				for(long xNum = x - 1; xNum <= x+1; xNum++)
				{
					for(long yNum = y-1; yNum <= y+1; yNum++)
					{
						neighbourVal = _gridLayer.read(xNum,yNum);
						if(SosUtil::fabs(neighbourVal - val)>=minThreshold )
						{
							thresholdPassed = true;						
						}
					}
				}

				if(thresholdPassed)
				{
					tempMap.updateGridRef(1,x,y);
				}
				else
				{
					tempMap.updateGridRef(0,x,y);
				}					
			}
		}
	}

	//the VoronoiDiagramGenerator is, obviously enough, used to generate a voronoi diagram.
	//When the voronoi algorithm is applied to the filtered map we've already generated
	//it calculates the lines, albeit in a very fragmented form.
	//After the voronoi diagram is finished generating, we have to perform quite a few
	//other operations to join all the horizontal/vertical lines together to reduce the
	//dimensionality of the map.
	VoronoiDiagramGenerator vdg;

	long xMin=0,xMax=0,yMin=0,yMax=0;
	tempMap.setDimensions(west,north,east,south);
	tempMap.getAllUpdatedDimensions(xMin,yMax,xMax,yMin);

	SosUtil::ensureSmaller(xMin,xMax);
	SosUtil::ensureSmaller(yMin,yMax);
	
	GridMap<float> mapToUse(1000,1,0);
	float * xValues = 0, *yValues = 0;
	long count = 0;
	
	val = 0;
	float* tempRowAbove = new float[xMax - xMin + 4];
	float* tempRowBelow = new float[xMax - xMin + 4];
	float* tempRowCentre = new float[xMax - xMin + 4];

	if(tempRowAbove == 0 || tempRowBelow == 0 || tempRowCentre == 0)
	{
		setErrorStrings(STD_ERROR_TITLE,STD_NOT_ENOUGH_MEMORY);
		if(tempRowAbove != 0)
			delete[] tempRowAbove;
		if(tempRowBelow != 0)
			delete[] tempRowBelow;
		if(tempRowCentre != 0)
			delete[] tempRowCentre;
		
		return false;
	}

	//to reduce the dimensionality of the problem, we only need the unoccupied cells 
	//that are next to occupied cells.  This will lead to incorrect edges being generated, 
	//but these can be filtered out later
	for(y = yMin; y<= yMax; y++)
	{
		tempMap.copyRow(tempRowCentre,y,xMin-1,xMax+1);
		tempMap.copyRow(tempRowAbove,y+1,xMin-1,xMax+1);
		tempMap.copyRow(tempRowBelow,y-1,xMin-1,xMax+1);

		for( x = xMin; x <= xMax; x++)
		{
			val = tempRowCentre[x - (xMin-1)];
			if(val == 1)//if the value is 1, just copy it over, since it'll be ignored anyway
			{
				mapToUse.updateGridRef(val,x,y);				
			}
			else if(x > xMin && x < xMax && y > yMin && y < yMax)
			{
				//only set a cell to 0 if it has a cell of value 1 next to it
				if(tempRowCentre[x+1- (xMin-1)] == 1 || tempRowCentre[x-1- (xMin-1)] == 1 ||
					tempRowBelow[x - (xMin-1)] == 1 || tempRowAbove[x - (xMin-1)] == 1 ||
					tempRowAbove[x+1 - (xMin-1)] == 1 || tempRowAbove[x -1- (xMin-1)] == 1 ||
					tempRowBelow[x -1- (xMin-1)] == 1 || tempRowBelow[x +1 - (xMin-1)] == 1)
				{
					mapToUse.updateGridRef(0,x,y);	
				}
				else
				{
					mapToUse.updateGridRef(1,x,y);
				}
			}
			else
			{
				mapToUse.updateGridRef(0,x,y);
			}
		}
	}

	mapToUse.setDimensions(west,north,east,south);

	//count the number of occupied cells - this is so we can know what size of an array
	//to declare
	for(y = yMin; y<= yMax; y++)
	{
		mapToUse.copyRow(tempRowCentre,y,xMin,xMax);
		for( x = xMin; x <= xMax; x++)
		{
			val = tempRowCentre[x - xMin];
			if(val == 0)
			{
				count++;
			}
		}
	}

	delete[] tempRow;
	delete[] tempRowAbove;
	delete[] tempRowBelow;
	tempRow = tempRowAbove = tempRowBelow = 0;

	xValues = new float[count];
	yValues = new float[count];

	//if we ran out of memory, exit gracefully
	if(xValues == 0 || yValues == 0)
	{
		setErrorStrings(STD_ERROR_TITLE,STD_NOT_ENOUGH_MEMORY);

		//clean up the arrays
		if(tempRowCentre != 0) delete [] tempRowCentre ;
		if(xValues != 0) delete[] xValues;
		if(yValues != 0) delete[] yValues; 
		return false;
	}	

	//get all cells marked with zero in mapToUse, and add them to the list of points to send
	//to the VoronoiDiagramGenerator
	count = 0;
	for(y = yMin; y<= yMax; y++)
	{
		mapToUse.copyRow(tempRowCentre,y,xMin,xMax);
		for(x = xMax; x >= xMin; x--)
		{
			val = tempRowCentre[x - xMin];
			if(val == 0)
			{
				xValues[count] = (float)x + 0.5f;
				yValues[count] = (float)y + 0.5f;
				count++;
			}
		}
	}

	delete[] tempRowCentre;
	tempRowCentre = 0;

	mapToUse.reset();

	//generate the voronoi diagram.  The last parameter, false, specifies that we are not interested
	//in vertex info, just the edges
	bool retval = vdg.generateVoronoi(xValues,yValues,count, (float)xMin, (float)xMax, 
										(float)yMin,(float)yMax,minDist,false);

	delete []xValues;
	delete []yValues;

	if(!retval)
	{
		return false;
	}	

	LineXYLayer line;
	PointXY p1,p2;
	line.layer = 0;
	line.type = OBJECT_TYPE_LINE;

	long x1 = 0, x2 = 0, y1 = 0, y2 = 0;

	List<LineXYLayer> edgeList, edgeListDontCheck;
	LineXYLayer edge, edge2, joinedEdge, discard;
	long counter = 0;
	long checkedEdgesCounter = 0;
	bool newEdge = false;

	Grid3DNoFile<ListUnordered<LineXYLayer >*> gridLines(100,1,0);
	ListUnordered<LineXYLayer >* lineList = 0,*lineList1=0,*lineList2 = 0;
	bool tryAgain = false;
	long lineCounter = 0;
	bool edgeIsAPoint = false, ignoreEdge = false;

	bool justDoPoints = false;

	vdg.resetIterator();

	//first get all the lines from the voronoi generator, and push them onto the 
	//map of lists, gridLists.  gridLists is a 2D hashmap, where each cell in the grid is a pointer
	//to a list of edges.  Each edge is pushed onto a list at each of it's endpoints.
	//If a line is neither a point, vertical nor horizontal, 
	//just push it onto the edgeListDontCheck list
	while(vdg.getNext(p1.x,p1.y,p2.x,p2.y))
	{
		x1 = (p1.x < 0) ? (long)(p1.x - 1):(long)p1.x;
		x2 = (p2.x < 0) ? (long)(p2.x - 1):(long)p2.x;
		y1 = (p1.y < 0) ? (long)(p1.y - 1):(long)p1.y;
		y2 = (p2.y < 0) ? (long)(p2.y - 1):(long)p2.y;

		//if the grid cell that either of the end points lands on is zero, then the edge should be discarded
		//since it is invalid - it only exists because of performance improvements
		if(tempMap.getGridRef(x1,y1) == 0 || tempMap.getGridRef(x2,y2) == 0)
			continue;

		edgeIsAPoint = (p1 == p2);

		edge.value = 1;		
		edge.pt1 = p1;
		edge.pt2 = p2;
		edge.layer = ++lineCounter;

		if(edgeIsAPoint)
		{
			edge.type = OBJECT_TYPE_RECTANGLE;
		}
		else
		{
			edge.type = OBJECT_TYPE_LINE;
		}

		if(p1.x != p2.x && p1.y != p2.y)
		{
			edgeListDontCheck.push(edge);
			continue;
		}

		lineList = gridLines.getGridRef((long)p1.x,(long)p1.y);	

		if(lineList == 0)
		{
			lineList = new ListUnordered<LineXYLayer >();
			gridLines.updateGridRef(lineList,(long)p1.x,(long)p1.y);
//			LOG<<"created a list at ("<<(long)p1.x<<","<<(long)p1.y<<")";
		}	

		lineList->push(edge);

		if(edgeIsAPoint || (long(edge.pt1.x) == long(edge.pt2.x) && long(edge.pt1.y) == long(edge.pt2.y)) )//if it's  point, only add it to a list once
		{
			continue;
		}

		lineList = gridLines.getGridRef((long)p2.x,(long)p2.y);	

		if(lineList == 0)
		{
			lineList = new ListUnordered<LineXYLayer >();
			gridLines.updateGridRef(lineList,(long)p2.x,(long)p2.y);
		}

		lineList->push(edge);
	}

	vdg.reset();	

	gridLines.getAllUpdatedDimensions(xMin,yMax,xMax,yMin);
	ListUnordered<LineXYLayer >** listArray = new ListUnordered<LineXYLayer >*[xMax - xMin +1];

	//delete all the points that share a cell with an actual line
	for(y = yMin; y<= yMax; y++)
	{
		gridLines.copyRow(listArray,y,xMin,xMax);
		for(x = xMin; x <= xMax; x++)
		{
			lineList = listArray[x - xMin];
			if(lineList != 0 && lineList->getListSize() > 1)
			{
				lineList->resetIterator();
				while(lineList->readNext(edge))
				{
					if(edge.pt1 == edge.pt2 && lineList->getListSize() > 1)
					{
						lineList->remove();
					}
				}

			}
	
		}
	}

	const int pointCheckArrSize = 4;
	const int direcX[pointCheckArrSize] = {-1,-1,1,1};
	const int direcY[pointCheckArrSize] = {-1,1,-1,1};
	const int oppositeDirec[pointCheckArrSize] = {3,2,1,0};
	LineXYLayer point;
	bool joinedWithOtherPoint = false;

	Grid3D<bool*> gridPointCheck(100,0,0);
	bool* array = 0;

	//go through the grid looking for points.  when a point is found, look around it in
	//diagonal directions.  If there is another point there, create a new line between them
	//and push it onto the edgeListDontCheck
	for(y = yMin ; y<= yMax; y++)
	{
		gridLines.copyRow(listArray,y,xMin,xMax);
		for(x = xMin; x <= xMax; x++)
		{
			lineList = listArray[x - xMin];
			if(lineList != 0)
			{
//				LOG<<"checking list at ("<<x<<","<<y<<") for points to join";
				lineList->resetIterator();
				while(lineList->readNext(edge))
				{
					if(edge.pt1 == edge.pt2)
					{
						joinedWithOtherPoint = false;
						//search around the cell
						for(int i = 0; i < pointCheckArrSize; i++)
						{							
							lineList1 = gridLines.getGridRef(x + direcX[i], y+direcY[i]);
														
							if(lineList1 != 0 && 
								lineList1->readHead(point) && 
								point.pt1 == point.pt2)
							{
								array = gridPointCheck.getGridRef(x,y);

								//if a line has already been created between these two points, skip it and keep going
								if(array != 0 && array[oppositeDirec[i]] == true)
									continue;

								edge.value = -2;
								lineList->replace(edge);//,edge);

								point.value = -2;
								lineList1->replace(point);

								point.pt1 = edge.pt1;
								point.layer = ++lineCounter;
								point.value = 1;
								point.type = OBJECT_TYPE_LINE;

								edgeListDontCheck.push(point);

								array = gridPointCheck.getGridRef(x + direcX[i],y+direcY[i]);
								if(array == 0)
								{
									array = new bool[pointCheckArrSize];
									for(int count = 0; count < pointCheckArrSize; count++)
										array[count] = 0;

									gridPointCheck.updateGridRef(array,x + direcX[i],y+direcY[i]);
								}

								array[oppositeDirec[i]] = true;								

								//remove the outer point, and this point from their lists
								joinedWithOtherPoint = true;								
							}							
						}	
						
						//if this point is not touched by another point, just puch it onto
						//the list of edges, and remove it from the list
						if(!joinedWithOtherPoint)
						{
							edgeList.push(edge);
							lineList->remove();
						}
					}
				}
			}
		}
	}

	
	//clean up the gridPointCheck grid
	gridPointCheck.getAllUpdatedDimensions(xMin,yMax,xMax,yMin);
	SosUtil::ensureSmaller(xMin,xMax);
	SosUtil::ensureSmaller(yMin,yMax);

	xMin--;//bool arrays may have been created one cell outside the boundary
	xMax++;//so expand the area we search by one cell in every direction
	yMin--;
	yMax++;

	bool ** boolArrays = new bool*[xMax - xMin +1];
	for(y = yMin ; y<= yMax; y++)
	{
		gridPointCheck.copyRow(boolArrays,y,xMin,xMax);
		for(x = xMin; x <= xMax; x++)
		{
			if(boolArrays[x - xMin] != 0)
			{
				delete[] boolArrays[x - xMin];
			}
		}
	}

	delete[] boolArrays;
	boolArrays = 0;
	gridPointCheck.reset();

	//now go through the grid and remove all points.
	for(y = yMin ; y<= yMax; y++)
	{
		gridLines.copyRow(listArray,y,xMin,xMax);
		for(x = xMin; x <= xMax; x++)
		{
			lineList = listArray[x - xMin];
			if(lineList != 0)
			{
				lineList->resetIterator();
				while(lineList->readNext(edge))
				{
					if(edge.pt1 == edge.pt2)
					{
						lineList->remove();
					}
				}
				
				if(lineList->getListSize() < 1)
				{	
					delete lineList;
					listArray[x - xMin] = 0;
					gridLines.updateGridRef(0,x,y);
				}
			}
		}
	}

	delete [] listArray;
	listArray = 0;

	long advance = 0;
	bool found = false;

	gridLines.getAllUpdatedDimensions(xMin,yMax,xMax,yMin);
	listArray = new ListUnordered<LineXYLayer >*[xMax - xMin +1];

	//At this point, all points should be gone from the lists in the grid, along with all
	//the lines that are neither horizontal nor vertical
	//So, now go through the grid of lists, and join up the HORIZONTAL lines
	for(y = yMin ; y<= yMax; y++)
	{
		gridLines.copyRow(listArray,y,xMin,xMax);
		for(x = xMin; x <= xMax; x++)
		{
			lineList = listArray[x - xMin];

			//if there is no list here, skip this cell, obviously enough
			if(lineList == 0)
				continue;

			if(lineList->getListSize() == 0)
			{
				delete lineList;
				listArray[x - xMin] = 0;
				gridLines.updateGridRef(0,x,y);
				continue;
			}

			found = false;
			//go through the list looking for a horizontal line
			lineList->resetIterator();
			while(!found && lineList->readNext(edge))
			{
				//only choose lines where there leftmost point lies on the current cell
				//otherwise they will already have been processed
				if(edge.pt1.y == edge.pt2.y && 
					(long)SosUtil::minVal(edge.pt1.x,edge.pt2.x) >= x &&
					SosUtil::getDist(edge.pt1,edge.pt2) > 0.01) //this gets rid of an annoying bug.
				{
					found  = true;
				}
			}

			//if there were no horizontal lines in the list, move one to the right
			if(!found)
			{
				continue;
			}

			//look at the list where the line ends to see if there is another line there 
			//that this can be joined to
			p1.y = edge.pt1.y;
			p1.x = SosUtil::maxVal(edge.pt1.x, edge.pt2.x);//p1.x is the X value most to the right

			lineList1 = gridLines.getGridRef((long)p1.x,(long)p1.y);

			if(lineList1 == 0)
			{
//				LOG<<"Error: the other list at ("<<(long)p1.x<<","<<(long)p1.y<<" should NOT be zero";
				continue;
			}

			found = false;
			lineList1->resetIterator();
			while(!found && lineList1->readNext(edge2))
			{
				if(edge2.layer != edge.layer &&		//if not the same edge as the orig edge
					edge2.pt1.y == edge.pt1.y &&	//if at the same Y level as the orig edge
					edge2.pt1.y == edge.pt2.y &&	//if horizontal
					SosUtil::fabs(SosUtil::minVal(edge2.pt1.x,edge2.pt2.x) - 
									SosUtil::maxVal(edge.pt1.x,edge.pt2.x)) < 0.01)
				{
					found = true;
				}
			}

			lineList2 = 0;
			//if there is no other edge to join to, go on to the next edge
			if(!found)
			{
				//if not found, look one more cell to the right, just in case :-)
				p1.x ++;
				lineList2 = gridLines.getGridRef((long)p1.x,(long)p1.y);

				if(lineList2 == 0)
				{
					continue;
				}

				found = false;
				lineList2->resetIterator();
				while(!found && lineList2->readNext(edge2))
				{
					if(edge2.layer != edge.layer  &&	//if not the same edge as the orig edge
						edge2.pt1.y == edge.pt1.y &&	//if at the same Y level as the orig edge
						edge2.pt1.y == edge.pt2.y &&	//if horizontal
					SosUtil::fabs(SosUtil::minVal(edge2.pt1.x,edge2.pt2.x) - 
									SosUtil::maxVal(edge.pt1.x,edge.pt2.x)) < 0.01)
					{
						found = true;
					}
				}
			}

			if(!found)
				continue;

			//now we have two lines that can be joined, so let's do it!!!
			joinedEdge.pt1.y = joinedEdge.pt2.y = edge.pt1.y; //both edges have the same Y value, so it doesn't matter which we use
			joinedEdge.pt1.x = SosUtil::minVal(edge.pt1.x,edge.pt2.x);//the leftmost point
			joinedEdge.pt2.x = SosUtil::maxVal(edge2.pt1.x,edge2.pt2.x);//the rightmost point
			
			joinedEdge.layer = ++lineCounter;
			joinedEdge.value = 1;

			lineList->remove();//remove orig edge from its list

			if(lineList2 != 0)
			{
				lineList2->remove();//we must have gotten the edge from one pos to the right
			}
			else
			{
				lineList1->remove();//remove the edge being joined to from its list
			}

			lineList1->popVal(edge,discard);//remove orig edge from list to the right
		
			lineList2 = gridLines.getGridRef((long)joinedEdge.pt2.x, (long)joinedEdge.pt2.y);

			if(lineList2 == 0)
			{
				LOG<<"Error!  lineList2 at "<<joinedEdge.pt2<<" should NOT be NULL";
				lineList2 = new ListUnordered<LineXYLayer >();
				gridLines.updateGridRef(lineList2,(long)joinedEdge.pt2.x, (long)joinedEdge.pt2.y);		
			}

			lineList2->popVal(edge2, discard);//remove edge being joined to from its edge

			lineList->push(joinedEdge);//push new edge onto orig list
			lineList2->push(joinedEdge);//push new edge onto its new list

			LOG<<"Created new edge = "<<joinedEdge;

			lineList->resetIterator();
			x--;//check this list again

		}
	}

	delete [] listArray;
	listArray = 0;

	//So, now go through the grid of lists, and join up the VERTICAL lines
	for(x = xMin; x <= xMax; x++)
	{
		for(y = yMin ; y<= yMax; y++)
		{
			lineList = gridLines.getGridRef(x,y);

			//if there is no list here, skip this cell, obviously enough
			if(lineList == 0)
				continue;

			if(lineList->getListSize() == 0)
			{
				delete lineList;				
				gridLines.updateGridRef(0,x,y);
				continue;
			}

			found = false;
			//go through the list looking for a horizontal line
			lineList->resetIterator();
			while(!found && lineList->readNext(edge))
			{
				//only choose lines where there leftmost point lies on the current cell
				//otherwise they will already have been processed
				if(edge.pt1.x == edge.pt2.x && 
					(long)SosUtil::minVal(edge.pt1.y,edge.pt2.y) >= y &&
					SosUtil::getDist(edge.pt1,edge.pt2) > 0.01)
				{
					found  = true;
				}
			}

			//if there were no horizontal lines in the list, move one to the right
			if(!found)
			{
				continue;
			}

			//look at the list where the line ends to see if there is another line there 
			//that this can be joined to
			p1.x = edge.pt1.x;
			p1.y = SosUtil::maxVal(edge.pt1.y, edge.pt2.y);//p1.y is the Y value most to the top

			lineList1 = gridLines.getGridRef((long)p1.x,(long)p1.y);

			if(lineList1 == 0)
			{
				continue;
			}

			found = false;
			lineList1->resetIterator();
			while(!found && lineList1->readNext(edge2))
			{
				if(edge2.layer != edge.layer &&		//if not the same edge as the orig edge
					edge2.pt1.x == edge.pt1.x &&	//if at the same Y level as the orig edge
					edge2.pt1.x == edge.pt2.x)		//if horizontal
				{
					found = true;
				}
			}

			//if there is no other edge to join to, go on to the next edge
			if(!found)
			{
				continue;
			}

			//now we have two lines that can be joined, so let's do it!!!
			joinedEdge.pt1.x = joinedEdge.pt2.x = edge.pt1.x; //both edges have the same Y value, so it doesn't matter which we use
			joinedEdge.pt1.y = SosUtil::minVal(edge.pt1.y,edge.pt2.y);//the leftmost point
			joinedEdge.pt2.y = SosUtil::maxVal(edge2.pt1.y,edge2.pt2.y);//the rightmost point
			
			joinedEdge.layer = ++lineCounter;
			joinedEdge.value = 1;

			lineList->remove();//remove orig edge from its list
			lineList1->remove();//remove the edge being joined to from its list
			lineList1->popVal(edge,discard);//remove orig edge from list to the right
		
			lineList2 = gridLines.getGridRef((long)joinedEdge.pt2.x, (long)joinedEdge.pt2.y);

			if(lineList2 == 0)
			{
				lineList2 = new ListUnordered<LineXYLayer >();
				gridLines.updateGridRef(lineList2,(long)joinedEdge.pt2.x, (long)joinedEdge.pt2.y);		
			}

			lineList2->popVal(edge2, discard);//remove edge being joined to from its edge

			lineList->push(joinedEdge);//push new edge onto orig list
			lineList2->push(joinedEdge);//push new edge onto its new list

			lineList->resetIterator();
			y--;//check this list again
		}
	}


	gridLines.getAllUpdatedDimensions(xMin,yMax,xMax,yMin);
	SosUtil::ensureSmaller(xMin,xMax);
	SosUtil::ensureSmaller(yMin,yMax);

	PointXYLong pt(0,0);
	bool inSameList  = false;

	listArray = new ListUnordered<LineXYLayer >*[xMax - xMin +2];


	//now copy all the lines from the grid lists to the edgeList
	for(y = yMin ; y<= yMax; y++)
	{
		gridLines.copyRow(listArray,y,xMin,xMax);
		for(x = xMin; x <= xMax; x++)
		{
			lineList = listArray[x - xMin];
			if(lineList != 0)
			{
				lineList->resetIterator();
				while(lineList->readNext(edge))
				{
					edgeList.push(edge);
					lineList->remove();

					if(long(edge.pt1.x) == x && long(edge.pt1.y) == y)
					{
						pt = edge.pt2;
					}
					else if(long(edge.pt2.x) == x && long(edge.pt2.y) == y)
					{
						pt = edge.pt1;
					}
					else
					{
						continue;
					}

					lineList1 = gridLines.getGridRef(pt.x,pt.y);

					if(lineList1 == lineList)
					{
						inSameList = true;
					}
					else
					{
						inSameList = false;
					}

					if(lineList1 != 0)
					{
						lineList1->popVal(edge,discard);
						if(lineList1->getListSize() < 1)
						{
							delete lineList1;
							lineList1 = 0;
							gridLines.updateGridRef(0,pt.x,pt.y);

							if(pt.y == y)
							{
								listArray[pt.x - xMin] = 0;
							}							
						}
						if(inSameList)
						{
							if(lineList1 != 0)
							{
								lineList->resetIterator();
							}
							else
							{
								//if we're in the same list, and we've deleted the list, then break out of the while loop
								break;
							}
						}
					}
				}

				if(listArray[x - xMin] != 0)
				{
					delete listArray[x - xMin];
					listArray[x - xMin] = 0;
					gridLines.updateGridRef(0,x,y);
				}
			}
		}
	}

	gridLines.reset();
	delete [] listArray;
	listArray = 0;

	edgeListDontCheck.resetIterator();
	while(edgeListDontCheck.readNext(edge))
	{
		edgeList.push(edge);
	}
	
	setViewGridMap(false);
	setViewVectorMap(true);

	LineXYLong lineLong;

	counter = 0;
//	disableRecordingChangedArea();
	while(edgeList.popHead(line))
	{
		counter ++;
			
		if(_viewGridMap)
		{
			line.layer = getNextLayer();
		}
		else
		{
			line.layer = getNextNegativeLayer();
		}

		line.value = valueToSet;

		//if the line is a single point, it will be represented by an unfilled rectangle 1 cell in size.
		if(line.pt1 == line.pt2)
		{
			if(line.pt1.x != (float)((long)line.pt1.x) && line.pt1.y != (float)((long)line.pt1.y) )
			{
				line.type = OBJECT_TYPE_RECTANGLE;
				line.pt1.x -= 0.499f;
				line.pt1.y -= 0.499f;
				line.pt2.x += 0.499f;
				line.pt2.y += 0.499f;
				gridToMm(line.pt1.x,line.pt1.y,lineLong.pt1.x,lineLong.pt1.y);
				gridToMm(line.pt2.x,line.pt2.y,lineLong.pt2.x,lineLong.pt2.y);

				line.pt1 = lineLong.pt1;
				line.pt2 = lineLong.pt2;
				setObject(line);
			}
		}
		else
		{
			//for some reason, the voronoi algorithm creates some tiny lines that are not quite points, 
			//so the slip through the algorithm earlier for joining points together.  Ignoring these lines
			//has pretty much no effect, except to reduce the dimensionality of the map nicely.
			if(SosUtil::getDist(line.pt1,line.pt2) >= 0.001)
			{
				line.type = OBJECT_TYPE_LINE;
				gridToMm(line.pt1.x,line.pt1.y,lineLong.pt1.x,lineLong.pt1.y);
				gridToMm(line.pt2.x,line.pt2.y,lineLong.pt2.x,lineLong.pt2.y);

				line.pt1 = lineLong.pt1;
				line.pt2 = lineLong.pt2;
				setObject(line);
			}		
		}		
	}	

	return true;
}


//This is a very simple conversion method that converts a grid map to a vector map 
//by fitting rectangles to the grid cells.
bool MapManager::convertGridToLine(float threshold)
{
	if(!hasMap())
		return false;

	Grid3DNoFile<bool> markMap(1000,1,0,4);
	GridMap<float>tempMap(1000,1,0);

	const int above = 0, below = 1, right = 2, left = 3;

	LineXYLong dim = _gridLayer.getDimensions();
	SosUtil::ensureSmaller(dim.pt1.x,dim.pt2.x);
	SosUtil::ensureSmaller(dim.pt2.y,dim.pt1.y);

	int x = 0, y = 0;
	long north = dim.pt1.y, south = dim.pt2.y, east = dim.pt2.x, west = dim.pt1.x;

	//copy the map out of the layer object
	west = _gridLayer.getDimensions(WEST);
	east = _gridLayer.getDimensions(EAST);
	north = _gridLayer.getDimensions(NORTH);
	south= _gridLayer.getDimensions(SOUTH);

	//any occupied cell that is surrounded on all sides by other occupied cells must be set to 0
	for(x = west+1; x< east; x++)
	{
		for(y = south+1; y< north; y++)
		{
			if(_gridLayer.read(x,y) >= threshold )
			{
				if(_gridLayer.read(x+1,y) >= threshold &&_gridLayer.read(x-1,y)>= threshold && 
					_gridLayer.read(x,y+1) >= threshold && _gridLayer.read(x,y-1)>= threshold)
				{
					tempMap.updateGridRef(0,x,y);
				}
				else
				{
					tempMap.updateGridRef(1,x,y);
				}
			}
			else
			{
				tempMap.updateGridRef(0,x,y);
			}
		}
	}

	long counter = 0;
	
	for(x = west; x <= east; x++)
	{
		for(y = south; y <= north; y++)
		{
			if(tempMap.getGridRef(x,y) >= threshold)
			{
				if(tempMap.getGridRef(x-1,y) < threshold)
				{
					markMap.updateGridRef(1,x,y,left);
					counter++;
				}
				if(tempMap.getGridRef(x+1,y) < threshold)
				{
					markMap.updateGridRef(1,x,y,right);
					counter++;
				}
				if(tempMap.getGridRef(x,y+1) < threshold)
				{
					markMap.updateGridRef(1,x,y,above);
					counter++;
				}
				if(tempMap.getGridRef(x,y-1) < threshold)
				{
					markMap.updateGridRef(1,x,y,below);
					counter++;
				}				
			}
		}
	}

	//reset the _gridLayer object - since this step is not possible to undo, 
	//we might as well clean up the memory used to store the undo points
	_gridLayer.deleteAllLayerInfo();

	long start = 0, finish = 0;
	bool buildingLine = false;
	LineXYLayer obj;
	LineXYLong lineLong;
	obj.value = 1;

//	disableRecordingChangedArea();

	//first create the horizontal lines
	for(y = south; y<= north; y++)
	{
		buildingLine = false;
		start  =west;
		finish= west;

		for(x = west; x <= east+1; x++)
		{
			if(markMap.getGridRef(x,y,above) != 0 || markMap.getGridRef(x,y,below) != 0 && x <= east)
			{
				if(buildingLine)
				{
					finish = x;
				}
				else	
				{
					buildingLine = true;
					start = x;
					finish =x;
				}
			}
			else
			{				
				//if we were building a line above this cell, finish it and add it to the list of objects
				if(buildingLine)
				{
					//if this single cell in standing on its own, make a rectangle out of it
					//if a cell above or below it is above the threshold, ignore it, as
					//it'll be caught in the y-pass
					if(start != finish || markMap.getGridRef(start,y,below) != 0 && markMap.getGridRef(start,y,above) != 0)
					{						
						obj.pt1.x = start + 0.01f;
						obj.pt2.x = finish + 0.99f;
						obj.pt1.y = y + 0.01f;
						obj.pt2.y = y + 0.99f;
						obj.type = OBJECT_TYPE_RECTANGLE;
						obj.layer = getNextLayer();	

						gridToMm(obj.pt1.x,obj.pt1.y,lineLong.pt1.x,lineLong.pt1.y);
						gridToMm(obj.pt2.x,obj.pt2.y,lineLong.pt2.x,lineLong.pt2.y);

						obj.pt1 = lineLong.pt1;
						obj.pt2 = lineLong.pt2;

						setObject(obj);
						start = finish+1;					
					}
				}
				buildingLine = false;
			}
		}
	}

	//now create the vertical lines
	for(x = west; x<= east; x++)
	{
		buildingLine = false;
		start  =south;
		finish= south;

		for(y = south; y <= north+1; y++)
		{
			if(markMap.getGridRef(x,y,left) != 0 || markMap.getGridRef(x,y,right) != 0 && y <= north )
			{
				if(buildingLine)
				{
					finish = y;
				}
				else	
				{
					buildingLine = true;
					start = y;
					finish =y;
				}
			}
			else
			{				
				//if we were building a line above this cell, finish it and add it to the list of objects
				if(buildingLine)
				{
					//if this single cell in standing on its own, ignore it - it's already been
					//caught in the x-pass
					if(start != finish)
					{
						if(markMap.getGridRef(x,start,left) != markMap.getGridRef(x,start,right))
						{
							start++;
						}
						if(markMap.getGridRef(x,finish,left) != markMap.getGridRef(x,finish,right))
						{				
							finish--;
						}

						if(start <= finish)
						{
							obj.pt1.y = start + 0.01f;
							obj.pt2.y = finish + 0.99f;
							obj.pt1.x = x + 0.01f;
							obj.pt2.x = x + 0.99f;
							obj.type = OBJECT_TYPE_RECTANGLE;
							obj.layer = getNextLayer();			
														
							gridToMm(obj.pt1.x,obj.pt1.y,lineLong.pt1.x,lineLong.pt1.y);
							gridToMm(obj.pt2.x,obj.pt2.y,lineLong.pt2.x,lineLong.pt2.y);

							obj.pt1 = lineLong.pt1;
							obj.pt2 = lineLong.pt2;
							
							setObject(obj);									
						}
						start = finish+1;
					}
				}
				buildingLine = false;
			}
		}
	}

	setViewVectorMap(true);
	setViewGridMap(false);
	
	return true;
}



LineXY	MapManager::LineXYLongToFloat(LineXYLong l, bool isRectangle = false)
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



bool MapManager::averageGridMap(char* filePath)
{
	if(filePath == 0)
		return false;

	GridMap<float> mapToAverage(1000,1,-1);
	GridMap<float> destinationMap(1000,1,0);

	LineXYLong currentMapSize = _gridLayer.getDimensions();

	GridMapParser parser;

	if(!parser.parseFile(filePath,&mapToAverage))
	{
		setErrorStrings("Error!","Failed to parse the file");
		return false;
	}

	long xMinOrig = _gridLayer.getDimensions(WEST); 					   
	long yMinOrig = _gridLayer.getDimensions(SOUTH);	
	long xMaxOrig = _gridLayer.getDimensions(EAST); 
	long yMaxOrig = _gridLayer.getDimensions(NORTH);
	
	long xMinNew = SosUtil::maxVal(xMinOrig,mapToAverage.getUpdatedDimensions(WEST));
	long xMaxNew = SosUtil::minVal(xMaxOrig,mapToAverage.getUpdatedDimensions(EAST));
	long yMinNew = SosUtil::maxVal(yMinOrig,mapToAverage.getUpdatedDimensions(SOUTH));
	long yMaxNew = SosUtil::minVal(yMaxOrig,mapToAverage.getUpdatedDimensions(NORTH));
	
	long xMin = SosUtil::minVal(xMinOrig,mapToAverage.getUpdatedDimensions(WEST));
	long xMax = SosUtil::maxVal(xMaxOrig,mapToAverage.getUpdatedDimensions(EAST));
	long yMin = SosUtil::minVal(yMinOrig,mapToAverage.getUpdatedDimensions(SOUTH));
	long yMax = SosUtil::maxVal(yMaxOrig,mapToAverage.getUpdatedDimensions(NORTH));	

		
	float newVal = 0;
	long layer = getNextLayer();

	float val1 = 0, val2 = 0;

	for(long x = xMin; x<= xMax; x++)
	{
		for(long y = yMax; y >= yMin; y--)
		{
			//if this cell is outside the loaded map, take the value from the other map
			//otherwise take the value from the loaded map
			if(!SosUtil::between(x,xMinOrig,xMaxOrig) || !SosUtil::between(y,yMinOrig,yMaxOrig))
			{
				val1 = mapToAverage.getGridRef(x,y);
			}
			else
			{
				val1 = _gridLayer.read(x,y);
			}
			if(!SosUtil::between(x,xMinNew,xMaxNew) || !SosUtil::between(y,yMinNew,yMaxNew))
			{
				val2 = val1;
			}
			else
			{
				val2 = mapToAverage.getGridRef(x,y);
			}			
			if(val1 == -1)
				val1 = val2;

			if(val2 == -1)
				val2 = val1;

			newVal = ((val1*_mapAverageCount) +	val2)/(_mapAverageCount + 1);									  
			
			destinationMap.updateGridRef(newVal,x,y);
		}
	}

	_gridLayer.initFromMap(&destinationMap,true);
	
	_mapAverageCount++;
	resetUndoInfo();
//	resetView();
	
//	setAllScreenToChanged();

	return true;
}


double MapManager::correlateMap(char* filePath)
{
	GridMap<float> tempMap(1000,1,0.5);
	GridMap<float> otherMap(1000,1,0.5);

	GridMapParser parser;

	if(!parser.parseFile(filePath,&otherMap))
		return false;

	getLatestGridMap(&tempMap);
	
	return otherMap.correlateMap(&tempMap);
}

double MapManager::mapScoreMap(char* filePath,bool justCompareOccAreas)
{
	GridMap<float> tempMap(1000,1,0.5);
	GridMap<float> otherMap(1000,1,0.5);

	GridMapParser parser;

	if(!parser.parseFile(filePath,&otherMap))
		return false;

	getLatestGridMap(&tempMap);	

	return otherMap.scoreMap(&tempMap,justCompareOccAreas);
}

bool MapManager::getLatestGridMap(GridMap<float>* mapToCopyInto)
{
	LOGENTRY("getLatestGridMap")
	long west = _gridLayer.getDimensions(WEST);
	long east = _gridLayer.getDimensions(EAST);
	long south = _gridLayer.getDimensions(SOUTH);
	long north = _gridLayer.getDimensions(NORTH);

	float val = 0;

	float* arr = new float[_gridLayer.getWidth() + 1];

	for(long y = south; y<= north; y++)
	{
		_gridLayer.copyRow(arr,y,west,east);
		for(long x = west; x<= east; x++)
		{
			val = arr[x - west];
			mapToCopyInto->updateGridRef(val,x,y);
		}
	}

	delete[] arr;

	mapToCopyInto->setDimensions(west,north,east,south);
	LOGEXIT("getLatestGridMap")
	return true;
}


bool MapManager::getAllObjects(std::queue<LineXYLayer>& listToCopyInto)
{
	if(!hasMap())
		return false;

	_listObjects.resetIterator();

	LineXYLayer object;

	while(_listObjects.readNext(object))
	{
		object.layer = 0;
		listToCopyInto.push(object);
	}

	return !listToCopyInto.empty();
}
bool MapManager::getRobots(std::queue<LineXYLayer>& listToCopyInto)
{
	if(!hasMap())
		return false;

	_listObjects.resetIterator();
	LineXYLayer object;

	while(_listObjects.readNext(object))
	{
		object.layer = 0;
		if(object.type == OBJECT_TYPE_ROBOT)
			listToCopyInto.push(object);
	}

	return !listToCopyInto.empty();
}



//NB: THE POPUP TO THE USER SHOULD BE IMPLEMENTED IN THE CHILD CLASS
void MapManager::setResolution(long res)
{
	//if the new resolution is the same as the old one, then do nothing
	if(res == _resolution)
	{
		return;
	}

	long oldRes = _resolution;
	

	resetUndoInfo();

	LineXYLayer obj;
	
	_listObjects.resetIterator();
	//first remove all vectors from the grid
	while(_listObjects.readNext(obj))
	{
		_gridLayer.popObject(obj,_resolution);
	}

	_resolution = res;

	_listObjects.resetIterator();

	while(_listObjects.readNext(obj))
	{
		_gridLayer.pushObject(obj,_resolution);		
	}
}

void MapManager::clearVoronoi()
{
	_listVoronoiLines.clear();
	_listVoronoiEdges.clear();
	_listVoronoiVertices.clear();
}

void MapManager::clearDelaunay()
{
	_listDelaunayLines.clear();
}

void MapManager::clearPaths()
{
	LOG<<"In clearPaths()"<<endl;
	_listPathLines.clear();

	List<PointXYLong>* pointList = 0;	

	while(_pathLists.popHead(pointList))
	{
		if(pointList != 0)
		{
			pointList->clear();
			delete pointList;
		}
	}
}

void MapManager::clearRobotRun()
{
	LOG<<"In clearRobotRun()"<<endl;
	_listPathLines.clear();

	ListUnordered<SosPose> * sensorReadingList = 0;
	while(_listRobotRunLists.popHead(sensorReadingList))
	{
		delete sensorReadingList;
	}

	sensorReadingList = 0;	
}


bool MapManager::clearRobots()
{
	if(_listObjects.getListSize() < 1)
		return false;

	_listObjects.resetIterator();

	LineXYLayer obj, discard;
	bool robotFound = false;

	while(_listObjects.readNext(obj))
	{
		if(obj.type == OBJECT_TYPE_ROBOT)
		{
			robotFound = true;
			_listObjects.remove();
		}
	}

	return robotFound;
}

void MapManager::clearVectors()
{
	LineXYLayer line;

	//if there are vectors to delete, then remove all "undo" memory
	//this is too big a change to be able to go back on
	if(_listObjects.getListSize() > 0)
	{
		_listUserActions.clear();
		_listUndoActions.clear();
		_listObjectsDeleted.clear();
		_listObjectsUndone.clear();
	}

	while(_listObjects.getListSize() > 0)
	{
		_listObjects.popHead(line);

		_gridLayer.popObject(line,_resolution);
	}

	_vectorBoundary.pt1.x = _gridLayer.getDimensions(WEST) *_resolution;
	_vectorBoundary.pt1.y = _gridLayer.getDimensions(SOUTH)*_resolution;
	_vectorBoundary.pt2.x = _gridLayer.getDimensions(EAST) *_resolution;
	_vectorBoundary.pt2.y = _gridLayer.getDimensions(NORTH)*_resolution;
}

//This clears the Grid Map, but not the Vectors that have been pushed on to it.
void MapManager::clearGridMap()
{
	long west = 0, north = 0, east = 0, south = 0;
	_gridLayer.getDimensions(west,north,east,south);
	_gridLayer.clearBaseMap(0);
	_gridLayer.reset(false);
	_gridLayer.crop(west, north,east,south);

	_listObjects.resetIterator();

	LineXYLayer obj;

	while(_listObjects.readNext(obj))
	{
		_gridLayer.pushObject(obj,_resolution);
	}

}

void MapManager::gridToMm(float gridX, float gridY, long& mmX, long& mmY)
{
	mmX = long(gridX * (float) _resolution);
	mmY = long(gridY * (float) _resolution);	
}

void MapManager::mmToGrid(long mmX, long mmY,float& gridX, float& gridY)
{
	gridX = (float)mmX / (float)_resolution;
	gridY = (float)mmY / (float)_resolution;
}

bool MapManager::generateCSpaceSimple(float min, float max, long distance)
{
	if(!hasMap())
	{
		return false;
	}

	LOG<<"generateCSpaceSimple got distance "<<distance;
	_gridLayer.generateCSpace(distance,min,max,_resolution);	

	resetUndoInfo();
	return true;
}


bool MapManager::thresholdMap(float min, float max)
{
	if(!hasMap())
	{
		return false;
	}

	GridMap<float> tempMap(1000,1,0);
	
	if(!hasMap())
		return false;

	long east, west, north,south;

	
	ListUnordered<LineXYLayer> tempList;
	LineXYLayer obj;

	while(_listObjects.getListSize() > 0)
	{
		_listObjects.popHead(obj);

		_gridLayer.popObject(obj,_resolution);
		
		if(!SosUtil::between(obj.value,min,max) || obj.value == -1)
		{
			obj.value = 0;
		}
		else
		{
			obj.value = 1;
		}

		obj.layer = getNextNegativeLayer();
		tempList.push(obj);
	}

	while(tempList.popHead(obj))
	{		
		_listObjects.push(obj);
	}
	
	_gridLayer.getDimensions(west,north,east,south);
	float val = 0;
	float *row = new float[east - west +1];

	for(long y = south; y<= north; y++)
	{
		_gridLayer.copyRow(row,y,west,east);
		for(long x = west; x<= east; x++)
		{
			val = row[x - west];
			
			if(!SosUtil::between(val,min,max) || val == -1)
			{
				tempMap.updateGridRef(0,x,y);
				
			}
			else
			{
				tempMap.updateGridRef(1,x,y);
			}			
		}
	}
	delete[] row;

	tempMap.setDimensions(west,north,east,south);
	
	_gridLayer.initFromMap(&tempMap,true);
	pushAllVectorsOntoGrid();
	
	resetUndoInfo();
	
	return true;
}

//flip all values of the grid map. If a value was 0, it will be 1, 0.8 ->0.2 etc
//Values of -1 stay the same
bool MapManager::negativeMap()
{
	LOG<<"In negativeMap()";
	if(!hasMap())
	{
		return false;
	}

	GridMap<float> tempMap(1000,1,0);
	
	if(!hasMap())
		return false;

	long east, west, north,south;

	
	ListUnordered<LineXYLayer> tempList;
	LineXYLayer obj;

	while(_listObjects.getListSize() > 0)
	{
		_listObjects.popHead(obj);

		_gridLayer.popObject(obj,_resolution);
		if(obj.value >= 0)
		{
			obj.value = 1 - obj.value;
		}
		obj.layer = getNextNegativeLayer();
		tempList.push(obj);
	}

	while(tempList.popHead(obj))
	{		
		_listObjects.push(obj);
	}
	
	_gridLayer.getDimensions(west,north,east,south);
	float val = 0;
	float *row = new float[east - west +1];

	for(long y = south; y<= north; y++)
	{
		_gridLayer.copyRow(row,y,west,east);
		for(long x = west; x<= east; x++)
		{
			val = row[x - west];

			if(val == -1)
			{
				tempMap.updateGridRef(val,x,y);
			}
			else
			{
				tempMap.updateGridRef(1 - val,x,y);
			}
		}
	}
	delete[] row;

	tempMap.setDimensions(west,north,east,south);
	
	_gridLayer.initFromMap(&tempMap,true);
	pushAllVectorsOntoGrid();
	
	resetUndoInfo();
	
//	resetView();
	
	return true;
}

bool MapManager::smoothMap(float min, float max)
{
	if(!hasMap())
	{
		return false;
	}

	long east, west, north,south;
	_gridLayer.getDimensions(west,north,east,south);

	List<PointXYZ> pointsToSmooth;
	PointXYZ pt;
	int count = 0;
	float total = 0;
	float val = 0;
	float *row = new float[east - west + 3];
	float *rowAbove = new float[east - west + 3];;
	float *rowBelow = new float[east - west + 3];

	for(long y = south; y<= north; y++)
	{
		_gridLayer.copyRow(row,y,west - 1,east + 1);
		_gridLayer.copyRow(rowBelow,y-1,west - 1,east + 1);
		_gridLayer.copyRow(rowAbove,y+1,west - 1,east + 1);
		for(long x = west; x<= east; x++)
		{
			//if(SosUtil::between(_gridLayer.read(x,y),min,max))
			if(SosUtil::between(row[x + 1],min,max))
			{
				count = 0;
				total = 0;

				val = row[x+2];
				if(!SosUtil::between(val,min,max))
				{
					count++;
					total += val;
				}

				val = row[x];
				if(!SosUtil::between(val,min,max))
				{
					count++;
					total += val;
				}

				val = rowBelow[x + 1];
				if(!SosUtil::between(val,min,max))
				{
					count++;
					total += val;
				}

				val = rowAbove[x+1];
				if(!SosUtil::between(val,min,max))
				{
					count++;
					total += val;
				}
				if(count >= 3)
				{
					pt.x = x;
					pt.y = y;
					pt.value = total/(float)count;
					pointsToSmooth.push(pt);
				}
			}
		}
	}

	if(pointsToSmooth.getListSize() == 0)
		return true;

	_gridLayer.deleteAllLayerInfo();
	resetUndoInfo();

	long layer = getNextLayer();
	while(pointsToSmooth.popHead(pt))
	{
		_gridLayer.push(pt.x,pt.y,layer,pt.value);
	}

	addUserAction(ACTION_SET_POINT);
	_listPoints.push(_latestLayer);

	return true;
}

//translate the map in millimetres in X and Y directions
bool MapManager::translateMap(long xDist, long yDist)
{
	if(!hasMap())
	{
		return false;
	}

	//the map can only be translated in whole grid cells, so normalize the distance
	//to be multiples of _resolution

	xDist -= xDist%_resolution;
	yDist -= yDist%_resolution;

	float xDistFloat = 0, yDistFloat = 0;

	//translate from millimetres to the Grid measurements
	mmToGrid(xDist,yDist,xDistFloat,yDistFloat);

//	xDist = (long)xDistFloat;
//	yDist = (long)yDistFloat;
	_gridLayer.translate((long)xDistFloat,(long)yDistFloat);
	
	resetUndoInfo();
	List<LineXYLayer> tempList;
	List<PointXY> tempPointList;
	List<LineXYLong> tempLongList;
	LineXYLayer obj;
	LineXYLong longObj;
	PointXY pt;

	while(_listObjects.popHead(obj))
	{
		obj.pt1.x += xDist;
		obj.pt2.x += xDist;
		obj.pt1.y += yDist;
		obj.pt2.y += yDist;
		tempList.push(obj);
	}
	while(tempList.popHead(obj))
	{
		_listObjects.push(obj);
	}

	while(_listPathLines.popHead(longObj))
	{
		longObj.pt1.x += xDist;
		longObj.pt2.x += xDist;
		longObj.pt1.y += yDist;
		longObj.pt2.y += yDist;
		tempLongList.push(longObj);
	}
	while(tempLongList.popHead(longObj))
	{
		_listPathLines.push(longObj);
	}

	while(_listVoronoiEdges.popHead(obj))
	{
		obj.pt1.x += xDist;
		obj.pt2.x += xDist;
		obj.pt1.y += yDist;
		obj.pt2.y += yDist;
		tempList.push(obj);
	}
	while(tempList.popHead(obj))
	{
		_listVoronoiEdges.push(obj);
	}

	while(_listVoronoiLines.popHead(obj))
	{
		obj.pt1.x += xDist;
		obj.pt2.x += xDist;
		obj.pt1.y += yDist;
		obj.pt2.y += yDist;
		tempList.push(obj);
	}
	while(tempList.popHead(obj))
	{
		_listVoronoiLines.push(obj);
	}

	while(_listVoronoiVertices.popHead(pt))
	{
		pt.x += xDist;
		pt.y += yDist;
		tempPointList.push(pt);
	}
	while(tempPointList.popHead(pt))
	{
		_listVoronoiVertices.push(pt);
	}

	_vectorBoundary.pt1.x += xDist;
	_vectorBoundary.pt2.x += xDist;
	_vectorBoundary.pt1.y += yDist;
	_vectorBoundary.pt2.y += yDist;

	return true;
}


//the parameters are in MM
bool MapManager::cropMap(long x1,long y1, long x2, long y2)
{
	if(!hasMap())
		return false;

	PointXYLong topLeft,bottomRight;	

	PointXY		topLeftGrid, bottomRightGrid;
	LineXYLong currentMapSize = _gridLayer.getDimensions();

	topLeft.x = x1;
	topLeft.y = y1;
	bottomRight.x = x2;
	bottomRight.y = y2;

	SosUtil::ensureSmaller(topLeft.x,bottomRight.x);//pt1 is top left
	SosUtil::ensureSmaller(bottomRight.y,topLeft.y);//pt2 is bottom right

	mmToGrid(topLeft.x,topLeft.y, topLeftGrid.x,topLeftGrid.y);
	mmToGrid(bottomRight.x,bottomRight.y,bottomRightGrid.x,bottomRightGrid.y);

	topLeftGrid.x = (float)SosUtil::roundDown(topLeftGrid.x);
	topLeftGrid.y = (float)SosUtil::roundUp(topLeftGrid.y);
	bottomRightGrid.x = (float)SosUtil::roundUp(bottomRightGrid.x);
	bottomRightGrid.y = (float)SosUtil::roundDown(bottomRightGrid.y);
	
	LOG<<"MapManager cropping ("<<topLeft.x<<","<<topLeft.y<<") -> ("<<bottomRight.x<<","<<bottomRight.y<<")";

	x1 = (long)topLeftGrid.x;
	y1 = (long)bottomRightGrid.y;
	x2 = (long)bottomRightGrid.x;
	y2 = (long)topLeftGrid.y;

	_gridLayer.crop(x1,y2,x2,y1);

	_mapMinX = x1;
	_mapMaxX = x2;
	_mapMinY = y1;
	_mapMaxY = y2;

	gridToMm(topLeftGrid.x,topLeftGrid.y,topLeft.x,topLeft.y);
	gridToMm(bottomRightGrid.x,bottomRightGrid.y,bottomRight.x,bottomRight.y);

	_vectorBoundary.pt1.x = topLeft.x;
	_vectorBoundary.pt1.y = bottomRight.y;
	_vectorBoundary.pt2.x = bottomRight.x;
	_vectorBoundary.pt2.y = topLeft.y;

	bool partial = true;
	bool pt1InRect = false,pt2InRect = false;

	resetUndoInfo();

	List<LineXYLayer> tempList;
	LineXY objLine, testLine;
	
	LineXYLayer obj;
	float slope = 0;
	double intersectX = 0,intersectY = 0;
	bool pt1set = false, pt2set = false;

	//remove all objects outside the cropped area
	while(_listObjects.popHead(obj))
	{
		if(objectInRect(obj,topLeft,bottomRight,partial))
		{
			//only a non-filled rectangle needs to be popped and repushed to the grid map
			//if its layer is positive because it needs a new line on one or more sides
			//the filled rectangle already has those cells painted, and the 
			//individual line is ok too
			switch(obj.type)
			{
			case OBJECT_TYPE_RECTANGLE:
				//if the rectangle has a positive layer number and is only partially in the cropped area
				//then pop it from the grid layer
				if(obj.layer > 0 && partial)
				{
					_gridLayer.popObject(obj,_resolution);
				}
				//fall through to the next case

			case OBJECT_TYPE_RECTANGLE_FILLED:
				SosUtil::ensureSmaller(obj.pt1.x,obj.pt2.x);
				SosUtil::ensureSmaller(obj.pt1.y,obj.pt2.y);
				obj.pt1.x = SosUtil::maxVal((long)obj.pt1.x,topLeft.x); //fix the left most point
				obj.pt1.y = SosUtil::maxVal((long)obj.pt1.y,bottomRight.y);//fix the bottom point
				obj.pt2.x = SosUtil::minVal((long)obj.pt2.x,bottomRight.x); //fix the right most point
				obj.pt2.y = SosUtil::minVal((long)obj.pt2.y,topLeft.y);//fix the top point

				if(obj.type == OBJECT_TYPE_RECTANGLE && obj.layer > 0 && partial)
					_gridLayer.pushObject(obj,_resolution);
				
				tempList.push(obj);
				break;
			case OBJECT_TYPE_ROBOT:
				if(SosUtil::between((long)obj.pt1.x,topLeft.x,bottomRight.x) &&
					SosUtil::between((long)obj.pt1.y,topLeft.y,bottomRight.y) &&
					SosUtil::between((long)obj.pt2.x,topLeft.x,bottomRight.x) &&
					SosUtil::between((long)obj.pt2.y,topLeft.y,bottomRight.y))
				{
					tempList.push(obj);	
				}

				break;
			case OBJECT_TYPE_LINE:
				if(!partial)
				{
					tempList.push(obj);
					break;
				}
				pt1InRect = (SosUtil::between((long)obj.pt1.x ,topLeft.x,bottomRight.x) && 
								SosUtil::between((long)obj.pt1.y ,topLeft.y,bottomRight.y));
				pt2InRect = (SosUtil::between((long)obj.pt2.x ,topLeft.x,bottomRight.x) && 
								SosUtil::between((long)obj.pt2.y ,topLeft.y,bottomRight.y));
				objLine.setPoints(obj.pt1.x,obj.pt1.y,obj.pt2.x,obj.pt2.y);
				pt1set = pt2set = false;

				//first test the left line
				testLine.pt1 = topLeft;
				testLine.pt2.y = bottomRight.y;
				testLine.pt2.x = topLeft.x;
				if(objLine.getIntersection(testLine,intersectX,intersectY,false))
				{
					LOG<<"Line intersects with the left line";
					if(pt1InRect)
					{
						obj.pt2.x = (float)intersectX;
						obj.pt2.y = (float)intersectY;
						pt2set = true;
					}
					else if(pt2InRect)
					{
						obj.pt1.x = (float)intersectX;
						obj.pt1.y = (float)intersectY;
						pt1set = true;
					}
					//if neither of the end points are in the rectangle, set pt1 to the intersection point
					else 
					{
						obj.pt1.x = (float)intersectX;
						obj.pt1.y = (float)intersectY;
						pt1set = true;
					}
				}

				//now test the top line
				testLine.pt2.x = bottomRight.x;
				testLine.pt2.y = topLeft.y;
				if(!(pt1set && pt2set) && objLine.getIntersection(testLine,intersectX,intersectY,false))
				{
					if(pt1InRect)
					{
						obj.pt2.x = (float)intersectX;
						obj.pt2.y = (float)intersectY;						
						pt2set = true;
					}
					else if(pt2InRect)
					{
						obj.pt1.x = (float)intersectX;
						obj.pt1.y = (float)intersectY;
						pt1set = true;
					}
					//if neither of the end points are in the rectangle, set pt1 to the intersection point
					else 
					{
						if(!pt1set)
						{
							obj.pt1.x = (float)intersectX;
							obj.pt1.y = (float)intersectY;
							pt1set = true;
						}
						else
						{
							obj.pt2.x = (float)intersectX;
							obj.pt2.y = (float)intersectY;
							pt2set = true;
						}						
					}
				}

				//now do the right line		
				testLine.pt1 = bottomRight;
				if(!(pt1set && pt2set) && objLine.getIntersection(testLine,intersectX,intersectY,false))
				{
					if(pt1InRect)
					{
						obj.pt2.x = (float)intersectX;
						obj.pt2.y = (float)intersectY;						
						pt2set = true;
					}
					else if(pt2InRect)
					{
						obj.pt1.x = (float)intersectX;
						obj.pt1.y = (float)intersectY;
						pt1set = true;
					}
					//if neither of the end points are in the rectangle, set pt1 to the intersection point
					else 
					{
						if(!pt1set)
						{
							obj.pt1.x = (float)intersectX;
							obj.pt1.y = (float)intersectY;
							pt1set = true;
						}
						else
						{
							obj.pt2.x = (float)intersectX;
							obj.pt2.y = (float)intersectY;
							pt2set = true;
						}						
					}
				}
				
				//now do the bottom line
				testLine.pt2.x = topLeft.x;
				testLine.pt2.y = bottomRight.y;
				if(!(pt1set && pt2set) && objLine.getIntersection(testLine,intersectX,intersectY,false))
				{
					if(pt1InRect)
					{
						obj.pt2.x = (float)intersectX;
						obj.pt2.y = (float)intersectY;						
						pt2set = true;
					}
					else if(pt2InRect)
					{
						obj.pt1.x = (float)intersectX;
						obj.pt1.y = (float)intersectY;
						pt1set = true;
					}
					//if neither of the end points are in the rectangle, set pt1 to the intersection point
					else 
					{
						if(!pt1set)
						{
							obj.pt1.x = (float)intersectX;
							obj.pt1.y = (float)intersectY;
							pt1set = true;
						}
						else
						{
							obj.pt2.x = (float)intersectX;
							obj.pt2.y = (float)intersectY;
							pt2set = true;
						}						
					}
				}
			
				tempList.push(obj);

				break;

			}			
		}
	}

	while(tempList.popHead(obj))
	{
		_listObjects.push(obj);
	}

	return true;
}

void MapManager::resetUndoInfo()
{
	LOGENTRY("resetUndoInfo");
	_listObjectsReplaced.clear();
	_listObjectsUndone.clear();
	_listObjectsDeleted.clear();
	_listPointsUndone.clear();
	_listUndoActions.clear();
	_listUserActions.clear();
	_listObjectsReplacedUndone.clear();
	LOGEXIT("resetUndoInfo");
}



bool MapManager::objectInRect(const LineXYLayer &obj, PointXYLong topLeft,PointXYLong bottomRight, bool& partiallyInRect)
{
	//this should give a quick answer.  If any end/corner point of an object is in 
	//the rectangle, return true.  This might screw up for ellipses, but right now
	//we don't support them
	if(SosUtil::between((long)obj.pt1.x  ,topLeft.x,bottomRight.x)  && 
		SosUtil::between((long)obj.pt1.y ,topLeft.y,bottomRight.y))
	{
		if(SosUtil::between((long)obj.pt2.x ,topLeft.x,bottomRight.x) &&
			SosUtil::between((long)obj.pt2.y ,topLeft.y,bottomRight.y))
		{
			partiallyInRect = false;
		}
		else
		{
			partiallyInRect = true;
		}
		return true;
	}
	if(SosUtil::between((long)obj.pt2.x ,topLeft.x,bottomRight.x) &&
		SosUtil::between((long)obj.pt2.y ,topLeft.y,bottomRight.y))
	{
		if(SosUtil::between((long)obj.pt1.x  ,topLeft.x,bottomRight.x)  && 
			SosUtil::between((long)obj.pt1.y ,topLeft.y,bottomRight.y))
		{
			partiallyInRect = false;
		}
		else
		{
			partiallyInRect = true;
		}
		return true;
	}

	partiallyInRect = true;

	bool tlxInRect,tlyInRect,brxInRect,bryInRect;
	LineXY objLine(obj.pt1,obj.pt2), testLine(0,0,0,0);
	double intersectX = 0,intersectY = 0;

	switch(obj.type)
	{
	case OBJECT_TYPE_RECTANGLE_FILLED:
	case OBJECT_TYPE_RECTANGLE:
	case OBJECT_TYPE_ROBOT:
		if(!SosUtil::between(obj.pt1.x/_resolution, (float)_mapMinX,(float)_mapMaxX) && 
			!SosUtil::between(obj.pt2.x/_resolution, (float)_mapMinX,(float)_mapMaxX))
			return false;
		if(!SosUtil::between(obj.pt1.y/_resolution, (float)_mapMinY,(float)_mapMaxY) && 
			!SosUtil::between(obj.pt2.y/_resolution, (float)_mapMinY,(float)_mapMaxY))
			return false;


		LOG<<"Object type is rect of some kind";
		tlxInRect = SosUtil::between((long)topLeft.x,(long)obj.pt1.x,(long)obj.pt2.x);
		tlyInRect = SosUtil::between((long)topLeft.y,(long)obj.pt1.y,(long)obj.pt2.y);
		brxInRect = SosUtil::between((long)bottomRight.x,(long)obj.pt1.x,(long)obj.pt2.x);
		bryInRect = SosUtil::between((long)bottomRight.y,(long)obj.pt1.y,(long)obj.pt2.y);


		if((tlxInRect == !brxInRect && tlyInRect && bryInRect) ||
			(tlyInRect == !bryInRect && tlxInRect && brxInRect))
		{
			LOG<<"Returning true 2: "<<tlxInRect<<","<<tlyInRect<<","<<brxInRect<<","<<bryInRect;
			return true;
		}
		if(tlxInRect && tlyInRect)
		{
			if(obj.type == OBJECT_TYPE_RECTANGLE_FILLED)
				return true;

			return !(brxInRect && bryInRect);
		}

		if(tlxInRect && bryInRect)
		{
			if(obj.type == OBJECT_TYPE_RECTANGLE_FILLED)
				return true;

			return !(brxInRect && tlyInRect);
		}

		if(brxInRect && tlyInRect)
		{
			if(obj.type == OBJECT_TYPE_RECTANGLE_FILLED)
				return true;

			return !(tlxInRect && bryInRect);
		}

		if(brxInRect && bryInRect)
		{
			if(obj.type == OBJECT_TYPE_RECTANGLE_FILLED)
				return true;
			return !(tlxInRect && tlyInRect);
		}

		if(SosUtil::between((long)topLeft.y,(long)obj.pt1.y,(long)obj.pt2.y) &&
			SosUtil::between(bottomRight.y,(long)obj.pt1.y,(long)obj.pt2.y) && 
			topLeft.x < SosUtil::minVal(obj.pt1.x,obj.pt2.x) &&
			bottomRight.x > SosUtil::maxVal(obj.pt1.x,obj.pt2.x))
		{
			return true;
		}

		if(SosUtil::between(topLeft.x,(long)obj.pt1.x,(long)obj.pt2.x) &&
			SosUtil::between(bottomRight.x,(long)obj.pt1.x,(long)obj.pt2.x) && 
			topLeft.y > SosUtil::maxVal(obj.pt1.y,obj.pt2.y) &&
			bottomRight.y < SosUtil::minVal(obj.pt1.y,obj.pt2.y))
		{
			return true;
		}

		return false;
	case OBJECT_TYPE_LINE:
		
		LOG<<"Object type line";
		//we already know that neither of the line's 2 points are in the rectangle
		//if the line intersects with the top, bottom, left or right lines of the rect within the rect, 
		//return true, otherwise false

		//first test the left line
		testLine.pt1 = topLeft;
		testLine.pt2.y = bottomRight.y;
		testLine.pt2.x = topLeft.x;
		if(objLine.getIntersection(testLine,intersectX,intersectY,false,0))
		{
			LOG<<"Returning true line 1";
			return true;
		}

		//now test the top line
		testLine.pt2.x = bottomRight.x;
		testLine.pt2.y = topLeft.y;
		if(objLine.getIntersection(testLine,intersectX,intersectY,false,0))
		{
			LOG<<"Returning true line 2";
			return true;
		}

		//now do the right line		
		testLine.pt1 = bottomRight;
		if(objLine.getIntersection(testLine,intersectX,intersectY,false,0))
		{
			LOG<<"Returning true line 3";
			return true;
		}

		//now do the bottom line
		testLine.pt2.x = topLeft.x;
		testLine.pt2.y = bottomRight.y;
		if(objLine.getIntersection(testLine,intersectX,intersectY,false,0))
		{
			LOG<<"Returning true line 4";
			return true;
		}

		return false;	
		
		break;
	}

	return false;
}

void MapManager::setViewVectorMap(bool viewVectorMap)
{
	if(viewVectorMap == false && getViewGridMap() == false)
	{
		setViewGridMap(true);
	}
	_viewVectorMap = viewVectorMap; 		
}

void MapManager::setErrorStrings(char* title, char* errorString)
{
	strcpy(_errorBuffer[0],title);
	strcpy(_errorBuffer[1],errorString);
	_hasError = true;
}

void MapManager::setErrorStrings(char* title, char* errorString1,char* errorString2)
{
	strcpy(_errorBuffer[0],title);
	strcpy(_errorBuffer[1],errorString1);
	strcat(_errorBuffer[1],errorString2);
	_hasError = true;
}

bool MapManager::getErrorStrings(char* title, char* errorString)
{
	if(!_hasError)
		return false;
	strcpy(title,_errorBuffer[0]);
	strcpy(errorString,_errorBuffer[1]);

	_hasError = false;
	return true;
}

bool MapManager::getDimensions(long& west, long&north,long&east, long&south)
{
	if(!hasMap())
		return false;

	_gridLayer.getDimensions(west,north,east,south);

	gridToMm(west,north,west,north);
	gridToMm(east,south,east,south);

	west = SosUtil::minVal(west,_vectorBoundary.pt1.x,_vectorBoundary.pt2.x);
	east = SosUtil::maxVal(east,_vectorBoundary.pt1.x,_vectorBoundary.pt2.x);
	north= SosUtil::maxVal(north,_vectorBoundary.pt1.y,_vectorBoundary.pt2.y);
	south= SosUtil::minVal(south,_vectorBoundary.pt1.y,_vectorBoundary.pt2.y);

	return true;

}

bool MapManager::getGridDimensions(long& west, long&north,long&east, long&south)
{
	if(!hasMap())
		return false;

	_gridLayer.getDimensions(west,north,east,south);

	return true;
}


IListReader<LineXYLayer>*	MapManager::getAllObjectsReader()
{
	return &_listObjects;	
}

IListReader<LineXY>*		MapManager::getVoronoiLinesReader()
{
	return &_listVoronoiLines;
}

IListReader<PointXY>*		MapManager::getVoronoiVerticesReader()
{
	return &_listVoronoiVertices;
}

IListReader<LineXY>*		MapManager::getDelaunayLinesReader()
{
	LOG<<"Returning a reference to the _listDelaunayLines, of size "<<_listDelaunayLines.getListSize();
	return &_listDelaunayLines;
}

IListReader<LineXYLong>*	MapManager::getPathLinesReader()
{
	return &_listPathLines;
}
ICopyRow2D<float>*			MapManager::getGridReader()
{
	return &_gridLayer;
}


bool MapManager::deleteObject(long objectNumber)
{
	if(!hasMap())
		return false;


	LineXYLayer obj;
	obj.layer = objectNumber;

	_listObjects.resetIterator();

	LineXYLayer foundObj;
	bool found = _listObjects.popVal(obj,foundObj);

	if(!found)
		return false;

	_listObjectsDeleted.push(foundObj);
	
	if(foundObj.layer > 0)
		_gridLayer.popObject(foundObj, _resolution);

	addUserAction(ACTION_DELETE_OBJECT);
	return true;
}

void MapManager::refreshVectorBoundary()
{
	if(!hasMap())
		return;

	_listObjects.resetIterator();
	LineXYLayer obj;
	
	while(_listObjects.readNext(obj))
	{
		_vectorBoundary.pt1.x = SosUtil::minVal(_vectorBoundary.pt1.x,(long)obj.pt1.x,(long)obj.pt2.x);
		_vectorBoundary.pt2.x = SosUtil::maxVal(_vectorBoundary.pt2.x,(long)obj.pt1.x,(long)obj.pt2.x);
		
		_vectorBoundary.pt1.y = SosUtil::minVal(_vectorBoundary.pt1.y,(long)obj.pt1.y,(long)obj.pt2.y);
		_vectorBoundary.pt2.y = SosUtil::maxVal(_vectorBoundary.pt2.y,(long)obj.pt1.y,(long)obj.pt2.y);		
	}
}

bool MapManager::getObject(long objectNumber, LineXYLayer& obj)
{
	if(!hasMap())
		return false;

	LineXYLayer objToFind;
	objToFind.layer = objectNumber;

	if(!_listObjects.readVal(objToFind,obj))
		return false;

	return true;
}

bool MapManager::replaceObject(long objectNumber, const LineXYLayer& object)
{
	if(!hasMap())
		return false;

	LineXYLayer newObject = object;
	LineXYLayer objToReplace;

	if(!_listObjects.popVal(object, objToReplace))
	{
		return false;
	}
	
	newObject.type = objToReplace.type;
	if(objToReplace.layer > 0)
	{
		_gridLayer.popObject(objToReplace, _resolution);
	}
	if(getViewGridMap())
	{		
		newObject.layer = getNextLayer();
		_gridLayer.pushObject(newObject, _resolution);

		LOG<<"Popped this object off the grid:"<<objToReplace;
		LOG<<"Pushed this object onto the grid: "<<newObject;
	}
	else
	{
		newObject.layer = getNextNegativeLayer();
	}	
	
	LayerValue<LineXYLayer> replacedObj, discard;
	replacedObj.layerNumber = newObject.layer;
	replacedObj.value = objToReplace;

	//keep the size of the list within a maximum
	while(_listObjectsReplaced.getListSize()  > NUM_UNDO_STEPS)
	{
		_listObjectsReplaced.popTail(discard);
	}

	_listObjectsReplaced.push(replacedObj);

	long minX = SosUtil::minVal(SosUtil::minVal(newObject.pt1.x,newObject.pt2.x),_vectorBoundary.pt1.x,_vectorBoundary.pt2.x);
	long maxX = SosUtil::maxVal(SosUtil::maxVal(newObject.pt1.x,newObject.pt2.x),_vectorBoundary.pt1.x,_vectorBoundary.pt2.x);
	long minY = SosUtil::minVal(SosUtil::minVal(newObject.pt1.y,newObject.pt2.y),_vectorBoundary.pt1.y,_vectorBoundary.pt2.y);
	long maxY = SosUtil::maxVal(SosUtil::maxVal(newObject.pt1.y,newObject.pt2.y),_vectorBoundary.pt1.y,_vectorBoundary.pt2.y);

	_vectorBoundary.setPoints(minX,minY,maxX,maxY);

	_listObjects.push(newObject);
	addUserAction(ACTION_REPLACE_OBJECT);

	return true;

}


bool MapManager::translateObjects(const std::vector<long>& objectNumbers, long xDist, long yDist, std::vector<long>& newObjectNumbers)
{
	long size = objectNumbers.size();

	LineXYLayer obj, foundObj ;
	ListUnordered<LineXYLayer> tempList;

	LayerValue<LineXYLayer> replacedObj;

	long minX, minY, maxX, maxY;
	minX = _vectorBoundary.pt1.x;
	maxX = _vectorBoundary.pt2.x;
	minY = _vectorBoundary.pt1.y;
	maxY = _vectorBoundary.pt2.y;
	SosUtil::ensureSmaller(minX,maxX);
	SosUtil::ensureSmaller(minY,maxY);

	for(int i = 0; i< size; i++)
	{
		//obj.layer = objectNumbers.at(i);
		obj.layer = objectNumbers[i];

		if(_listObjects.popVal(obj,foundObj))
		{
			if(foundObj.layer > 0)
			{
				_gridLayer.popObject(foundObj,_resolution);
			}
			if(getViewGridMap())
			{
				
				foundObj.layer = getNextLayer();
			}
			else
				foundObj.layer = getNextNegativeLayer();

			replacedObj.layerNumber = foundObj.layer;
			replacedObj.value = foundObj;
			_listObjectsReplaced.push(replacedObj);
			addUserAction(ACTION_REPLACE_OBJECT);

			foundObj.pt1.x += xDist;
			foundObj.pt2.x += xDist;
			foundObj.pt1.y += yDist;
			foundObj.pt2.y += yDist;
			tempList.push(foundObj);

			newObjectNumbers.push_back(foundObj.layer);

			minX = SosUtil::minVal(minX,foundObj.pt1.x,foundObj.pt2.x);
			maxX = SosUtil::maxVal(maxX,foundObj.pt1.x,foundObj.pt2.x);
			minY = SosUtil::minVal(minY,foundObj.pt1.y,foundObj.pt2.y);
			maxY = SosUtil::maxVal(maxY,foundObj.pt1.y,foundObj.pt2.y);
		}
	}

	while(tempList.popHead(foundObj))
	{
		_listObjects.push(foundObj);
		if(getViewGridMap())
		{
			_gridLayer.pushObject(foundObj,_resolution);
		}
	}

	_vectorBoundary.setPoints(minX,minY,maxX,maxY);

	return true;
}


bool MapManager::generateDelaunay(float threshold1, float threshold2, float minDistance)
{
	LOGENTRY("generateDelaunay")
	if(!hasMap())
		return false;

	SosUtil::ensureSmaller(threshold1, threshold2);

	LineXYLayer pushLine, tempLine;
	bool retval = false;

	LOG<<"generateDelaunay() about to add all vectors to grid map";
	
	//if the gridmap is not currently being viewed, go through all the vectors, and 
	//add any vector with a negative layer number to the grid
	if(_viewGridMap == false)
	{
		LOG<<"generateDelaunay() about to add all vectors to grid map->_viewGridMap = "<<_viewGridMap<<endl;
		pushAllVectorsOntoGrid();
		LOG<<"generateDelaunay() finished adding all vectors to grid map"<<endl;		
	}

	LOG<<"Creating a VoronoiDiagramGenerator object";

	VoronoiDiagramGenerator vdg;

	long xMin=0,xMax=0,yMin=0,yMax=0;
	_gridLayer.getDimensions(xMin,yMax,xMax,yMin);

	SosUtil::ensureSmaller(xMin,xMax);
	SosUtil::ensureSmaller(yMin,yMax);

	LOG<<"Boundaries for Delaunay are ("<<xMin<<","<<yMin<<") -> ("<<xMax<<","<<yMax<<")";

	float * xValues = 0, *yValues = 0;
	long count = 0;
	long x = 0, y = 0;
	float val = 0;
	List<PointXYLong> points;
	points.setModeQueue();
	PointXYLong ptLong;

	long mapWidth = xMax - xMin;
	float *row = new float[mapWidth + 1];
	
	//initialise the array
	for(int i = 0; i< mapWidth+1; i++)
	{
		row[i] = 0;
	}

	//do one scan through the current map.  If a cell within the two thresholds does not touch a cell
	//not between the thresholds, then ignore it, as it is surrounded by other cells similar to it.
	//Otherwise push the (x,y) position of the cell onto the list of points we're interested in
	
	LOG<<"Going to go through the grid map, looking for points to include in the graph"<<endl;
	for(y = yMin; y<= yMax; y++)
	{
		_gridLayer.copyRow(row,y,xMin,xMax);
		for(x = 0; x <= mapWidth; x++)
		{
			val = row[x];
			if(val < 0)
			{
				val = threshold1;//all negative values fall within the threshold
			}
			if(SosUtil::between(val,threshold1,threshold2))
			{
				ptLong.x = x + xMin;
				ptLong.y = y;

				points.push(ptLong);
			}
		}
	}
	
	//copy the voronoi cells into the two arrays to pass to the VoronoiDiagramGenerator
	count = points.getListSize();

	LOG<<"generateDelaunay() about create two float arrays of size "<<count<<endl;;
	xValues = new float[count];
	yValues = new float[count];

	if(xValues == 0 || yValues == 0)
	{
		LOG<<"generateDelaunay() returning false because it couldn't create two float arrays of size "<<count;
		if(xValues != 0) delete[] xValues;
		if(yValues != 0) delete[] yValues; 
		return false;
	}
	
	count = 0;
	while(points.popHead(ptLong))
	{
		xValues[count] = (float)ptLong.x+ 0.5f;
		yValues[count] = (float)ptLong.y+ 0.5f;
		count++;

	}

	
	LOG<<"generateDelaunay() Finished copying "<<count<<" values into float arrays";
	
	LOG<<"About to call generateDelaunay()";
	
	vdg.setGenerateDelaunay(true);
	vdg.setGenerateVoronoi(false);

	//generate the voronoi diagram
	retval = vdg.generateVoronoi(xValues,yValues,count, (float)xMin, (float)xMax, 
										(float)yMin,(float)yMax,minDistance);

	LOG<<"generateDelaunay() Finished generating the voronoi diagram";

	//clean up the two arrays
	delete []xValues;
	delete []yValues;

	LOG<<"generateDelaunay() Finished deleting the two float arrays";

	LOG<<"The generateVoronoi call on the VoronoiDiagramGenerator returned "<<retval<<endl;

	if(!retval)
	{
		return false;
	}

	LineXY line;

	float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
	long x1L = 0, x2L = 0,y1L = 0,y2L = 0;

	vdg.resetDelaunayEdgesIterator();

	_listDelaunayLines.clear();

	while(vdg.getNextDelaunay(x1,y1,x2,y2))
	{
		gridToMm(x1,y1,x1L, y1L);
		gridToMm(x2,y2,x2L,y2L);
		line.setPoints(x1L,y1L,x2L,y2L);

		_listDelaunayLines.push(line);			
	}	
	
	LOG<<"Pushed "<<_listDelaunayLines.getListSize()<<" lines onto the delaunay lines list";

	LOGEXIT("generateVoronoi")
	return true;
}


bool MapManager::generateVoronoi(float threshold1, float threshold2, float minDistance)
{
	LOGENTRY("generateVoronoi")
	if(!hasMap())
		return false;

	SosUtil::ensureSmaller(threshold1, threshold2);

	LineXYLayer pushLine, tempLine;
	bool retval = false;

	LOG<<"generateVoronoi() about to add all vectors to grid map";
	
	//if the gridmap is not currently being viewed, go through all the vectors, and 
	//add any vector with a negative layer number to the grid
	if(_viewGridMap == false)
	{
		LOG<<"generateVoronoi() about to add all vectors to grid map->_viewGridMap = "<<_viewGridMap<<endl;
		pushAllVectorsOntoGrid();
		LOG<<"generateVoronoi() finished adding all vectors to grid map"<<endl;		
	}

	LOG<<"getLatestGridMap returned "<<retval;

	LOG<<"Creating a VoronoiDiagramGenerator object";

	VoronoiDiagramGenerator vdg;

	long xMin=0,xMax=0,yMin=0,yMax=0;
	_gridLayer.getDimensions(xMin,yMax,xMax,yMin);

	LOG<<"Boundaries for voronoi are ("<<xMin<<","<<yMin<<") -> ("<<xMax<<","<<yMax<<")";

	float * xValues = 0, *yValues = 0;
	long count = 0;
	long x = 0, y = 0;
	float val = 0;
	List<PointXYLong> points;
	points.setModeQueue();
	PointXYLong ptLong;


	//do one scan through the current map.  If a cell within the two thresholds does not touch a cell
	//not between the thresholds, then ignore it, as it is surrounded by other cells similar to it.
	//Otherwise push the (x,y) position of the cell onto the list of points we're interested in
	for( x = xMin; x <= xMax; x++)
	{
		for(y = yMin; y<= yMax; y++)
		{
			val = _gridLayer.read(x,y);
			if(val < 0)
			{
				val = threshold1;//all negative values fall within the threshold
			}
			if(SosUtil::between(val,threshold1,threshold2))
			{
				if(!SosUtil::between(_gridLayer.read(x +1, y ),threshold1,threshold2) ||
					!SosUtil::between(_gridLayer.read(x-1 , y ),threshold1,threshold2) ||
					!SosUtil::between(_gridLayer.read(x+1 , y+1 ),threshold1,threshold2) ||
					!SosUtil::between(_gridLayer.read(x+1 , y-1 ),threshold1,threshold2) ||
					!SosUtil::between(_gridLayer.read(x , y +1),threshold1,threshold2) ||
					!SosUtil::between(_gridLayer.read(x , y -1),threshold1,threshold2) ||
					!SosUtil::between(_gridLayer.read(x -1, y +1),threshold1,threshold2) ||
					!SosUtil::between(_gridLayer.read(x -1, y -1),threshold1,threshold2))
				{
					ptLong.x = x;
					ptLong.y = y;
					points.push(ptLong);
				}
				//count++;
			}
		}
	}
	//copy the voronoi cells into the two arrays to pass to the VoronoiDiagramGenerator
	count = points.getListSize();

	LOG<<"generateVoronoi() about create two float arrays of size "<<count<<endl;;
	xValues = new float[count];
	yValues = new float[count];

	if(xValues == 0 || yValues == 0)
	{
		LOG<<"generateVoronoi() returning false because it couldn't create two float arrays of size "<<count;
		if(xValues != 0) delete[] xValues;
		if(yValues != 0) delete[] yValues; 
		return false;
	}
	
	count = 0;
	while(points.popHead(ptLong))
	{
		xValues[count] = (float)ptLong.x+ 0.5f;
		yValues[count] = (float)ptLong.y+ 0.5f;
		count++;
	}
	
	LOG<<"generateVoronoi() Finished copying "<<count<<" values into float arrays";
	
	LOG<<"About to call generateVoronoi()";
	
	vdg.setGenerateDelaunay(false);
	vdg.setGenerateVoronoi(true);

	//generate the voronoi diagram
	retval = vdg.generateVoronoi(xValues,yValues,count, (float)xMin, (float)xMax, 
										(float)yMin,(float)yMax,minDistance);

	LOG<<"generateVoronoi() Finished generating the voronoi diagram";

	//clean up the two arrays
	delete []xValues;
	delete []yValues;

	LOG<<"generateVoronoi() Finished deleting the two float arrays";

	LOG<<"The generateVoronoi call on the VoronoiDiagramGenerator returned "<<retval<<endl;

	if(!retval)
	{
		return false;
	}

	LineXY line;

	float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
	long x1L = 0, x2L = 0,y1L = 0,y2L = 0;

	
	//go through the lines in the graph - if either of the two ends of a line is in a cell that has
	//a value between the two thresholds, ignore the line, since it only exists because of the 
	//performance enhancement done earlier.  Otherwise store it in the list of voronoi lines
	LOG<<"generateVoronoi() about to iterate through all edges in the voronoi diagram";

	_listVoronoiLines.clear();
	_listVoronoiEdges.clear();
	_listVoronoiVertices.clear();

	vdg.resetIterator();

	while(vdg.getNext(x1,y1,x2,y2))
	{
		x1L = (x1 < 0) ? (long)(x1 - 1):(long)x1;
		x2L = (x2 < 0) ? (long)(x2 - 1):(long)x2;
		y1L = (y1 < 0) ? (long)(y1 - 1):(long)y1;
		y2L = (y2 < 0) ? (long)(y2 - 1):(long)y2;

		if(!SosUtil::between(_gridLayer.read(x1L,y1L),threshold1,threshold2) && 
			!SosUtil::between(_gridLayer.read(x2L,y2L),threshold1,threshold2))
		{
			gridToMm(x1,y1,x1L, y1L);
			gridToMm(x2,y2,x2L,y2L);
			line.setPoints(x1L,y1L,x2L,y2L);

			_listVoronoiLines.push(line);
		}
	}	
	
	vdg.resetVertexPairIterator();

	//go through the vertex pairs in the graph - if either of the two ends of a line is in a cell that has
	//a value between the two thresholds, ignore the line, since it only exists because of the 
	//performance enhancement done earlier.  Otherwise store it in the list of voronoi vertices
	while(vdg.getNextVertexPair(x1,y1,x2,y2))
	{
		x1L = (x1 < 0) ? (long)(x1 - 1):(long)x1;
		x2L = (x2 < 0) ? (long)(x2 - 1):(long)x2;
		y1L = (y1 < 0) ? (long)(y1 - 1):(long)y1;
		y2L = (y2 < 0) ? (long)(y2 - 1):(long)y2;

		if(!SosUtil::between(_gridLayer.read(x1L,y1L),threshold1,threshold2) && 
			!SosUtil::between(_gridLayer.read(x2L,y2L),threshold1,threshold2))
		{
			line.setPoints(x1,y1,x2,y2);
			_listVoronoiEdges.push(line);
		}
	}	

	
	vdg.resetVerticesIterator();
	PointXY pt;

	//go through the vertices in the graph - if either of the two ends of a line is in a cell that has
	//a value between the two thresholds, ignore the line, since it only exists because of the 
	//performance enhancement done earlier.  Otherwise store it in the list of voronoi vertices
	while(vdg.getNextVertex(pt.x,pt.y))
	{
		x1L = (pt.x < 0) ? (long)(pt.x - 1):(long)pt.x;
		y1L = (pt.y < 0) ? (long)(pt.y - 1):(long)pt.y;

		if(!SosUtil::between(_gridLayer.read(x1L,y1L),threshold1,threshold2))
		{
			_listVoronoiVertices.push(pt);
		}
	}
	
	LOG<<"Pushed "<<_listVoronoiLines.getListSize()<<" lines onto the voronoi lines list";
	LOG<<"Pushed "<<_listVoronoiVertices.getListSize()<<" vertices onto the voronoi vertices list";
	LOG<<"Pushed "<<_listVoronoiEdges.getListSize()<<" edges onto the voronoi edges list";

	LOGEXIT("generateVoronoi")
	return true;
}

void MapManager::cancelBulkJob()
{
	_bulkOperationCancelled = true;
}

bool MapManager::jobCompletedSuccessfully()
{
	return _bulkOperationSuccessful;
}