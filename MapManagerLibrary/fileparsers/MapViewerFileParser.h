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

#ifndef MAPVIEWER_FILE_PARSER_H
#define MAPVIEWER_FILE_PARSER_H

#include "SosUtil.h"
#include "SosList.h"
#include "GridMap.h"
#include <stdlib.h>

#include "../logger/Logger.h"

class MapViewerFileParser
{
public:
	MapViewerFileParser()
	{
		GET_FILE_LOG
		_listObjects.setModeOrderedAsc();
		_myMap = 0;

		reset();

	}

	void reset()
	{
		_hasGridMap = _hasVectors = false;
	
		if(_myMap != 0)
		{
			delete _myMap;
			_myMap = 0;
		}
		_listObjects.clear();
		minX= maxX=minY=maxY=0;
		_minMaxSet = false;
		_resolution = 100;
	}

	bool parseMapViewerMap(char* fileName)
	{
		LOG<<"In parseMapViewerMap";
		reset();

		if(fileName == 0)
			return false;

		ifstream in;
		in.open(fileName);
		if(!in.is_open())
			return false;

		_resolutionDisabled = false;

		//resolution_off

		const int numEntries = 13;
		char validEntriesStatic[numEntries][20]={"line","rect","rectfill","west","east","north",
												"south","data","gridmap","vectorobjects", "robot","resolution","resolution_off"};

		char **validEntries = 0;
		int i = 0;//used in for loops

		validEntries = new char*[numEntries];
		for( i = 0; i< numEntries; i++)
		{
			validEntries[i] = validEntriesStatic[i];
		}

		const int STATE_LINE = 0, STATE_RECT = 1, STATE_RECTFILLED = 2,STATE_WEST = 3,
			STATE_EAST = 4,STATE_NORTH = 5,STATE_SOUTH = 6,STATE_DATA = 7,STATE_GRIDMAP = 8,
			STATE_VECTORTITLE=9, STATE_ROBOT = 10, STATE_RESOLUTION = 11, STATE_RESOLUTION_OFF = 12;

		int currentState = -1;

		char buffer[250];
		bool fileIsBad = false, readObject = false;
		int entryType = -1;

		long west = 0, east = 0,south = 0, north = 0;
		bool gotWest = false, gotEast = false, gotNorth = false, gotSouth = false;
		float value = 0;

		long x = 0, y = 0; //used in for loop
		long numToSkip = 0;

		LOGCODE long gridCellCounter = 0;

		LineXYLayer object;

		in>>buffer;

		while(!fileIsBad && !in.eof())
		{
			LOG<<"Got token "<<buffer;
			entryType = SosUtil::stringInArrayNoCase(buffer,validEntries,numEntries);
			readObject = false;

			switch(entryType)
			{
			case STATE_GRIDMAP:
				currentState = STATE_GRIDMAP;
				break;
			case STATE_WEST:
				if(currentState != STATE_GRIDMAP)
				{
					fileIsBad = true;
					break;
				}

				in>>buffer;
				if(!SosUtil::is_integer(buffer))
				{
					fileIsBad = true;
					break;
				}

				west = atol(buffer);
				gotWest = true;

				break;
			case STATE_EAST:
				if(currentState != STATE_GRIDMAP)
				{
					fileIsBad = true;
					break;
				}

				in>>buffer;
				if(!SosUtil::is_integer(buffer))
				{
					fileIsBad = true;
					break;
				}

				east = atol(buffer);
				gotEast = true;
				break;
			case STATE_NORTH:
				if(currentState != STATE_GRIDMAP)
				{
					fileIsBad = true;
					break;
				}

				in>>buffer;
				if(!SosUtil::is_integer(buffer))
				{
					fileIsBad = true;
					break;
				}

				north = atol(buffer);
				gotNorth = true;
				break;
			case STATE_SOUTH:
				if(currentState != STATE_GRIDMAP)
				{
					fileIsBad = true;
					break;
				}
				in>>buffer;
				if(!SosUtil::is_integer(buffer))
				{
					fileIsBad = true;
					break;
				}

				south = atol(buffer);
				gotSouth = true;
				break;
			case STATE_DATA:
				LOG<<"In STATE_DATA"; 
				if(!gotWest || !gotEast || !gotSouth || !gotNorth)
				{
					LOG<<"Set file to bad because haven't got all dimensions";
					fileIsBad = true;
					break;
				}

				LOGCODE gridCellCounter = 0;

				_myMap = new GridMap<float>(1000,0,0);

				for(x = west;!fileIsBad &&  x<= east; x++)
				{
					for(y = south; !fileIsBad && y<= north; y++)
					{
						in>>buffer;
						if(!SosUtil::is_numeric(buffer))
						{
							if(buffer[0] != '[')// || buffer[strlen(buffer)-1]!=']')
							{
								fileIsBad = true;
								LOG<<"Set file to bad because buffer[0]="<<buffer[0]<<"and buffer["<<strlen(buffer)-1<<"]="<<buffer[strlen(buffer)-1]<<", when buffer = "<<buffer;
									
								break;
							}
							else
							{
								if(buffer[strlen(buffer)-1]==']')
									buffer[strlen(buffer)-1] = '\0';//end the string
								
								if(!SosUtil::is_integer(buffer + 1))
								{
									fileIsBad = true;
									LOG<<"Set file to bad because the number in brackets is not an integer ("<<buffer<<")";
									break;
								}
								numToSkip = atol(buffer+1);

								in>>buffer;

								if(!SosUtil::is_numeric(buffer))
								{
									fileIsBad = true;
									LOG<<"Set file to bad because the number after the brackets is non_numeric ("<<buffer<<")";
									
									break;	
								}
								value = (float)atof(buffer);

								value = SosUtil::minVal(1,value);
								
								if(value < 0 && value != -1)
									value = 0;

								for(long count = 0; count < numToSkip && y<= north; count++)
								{
									_myMap->updateGridRef(value,x,y);
									y++;
									LOGCODE gridCellCounter++;
								}
								y--;
							}
						}
						else
						{
							value = (float)atof(buffer);
							_myMap->updateGridRef(value,x,y);
							LOGCODE gridCellCounter++;
						}
					}
				}

				LOG<<"Read "<<gridCellCounter<<" cells";
				_hasGridMap = !fileIsBad;

				if(!fileIsBad)
				{
					_myMap->setDimensions(west,north,east,south);
				}

				break;
			case STATE_VECTORTITLE:
				currentState = STATE_VECTORTITLE;
				break;
			case STATE_LINE:
				if(currentState != STATE_VECTORTITLE)
				{
					fileIsBad = true;
					break;
				}
				readObject = true;
				object.type = OBJECT_TYPE_LINE;

				break;
			case STATE_RECT:
				if(currentState != STATE_VECTORTITLE)
				{
					fileIsBad = true;
					break;
				}
				readObject = true;
				object.type = OBJECT_TYPE_RECTANGLE;

				break;
			case STATE_RECTFILLED:
				if(currentState != STATE_VECTORTITLE)
				{
					fileIsBad = true;
					break;
				}
				readObject = true;
				object.type = OBJECT_TYPE_RECTANGLE_FILLED;

				break;
			case STATE_ROBOT:
				if(currentState != STATE_VECTORTITLE)
				{
					fileIsBad = true;
					break;
				}
				readObject = true;
				object.type = OBJECT_TYPE_ROBOT;

				break;
			case STATE_RESOLUTION:
				in>>buffer;

				if(!SosUtil::is_integer(buffer))
				{
					fileIsBad = true;
					LOG<<"Set file to bad because the number after the brackets is non_numeric ("<<buffer<<")";
					
					break;	
				}

				_resolution = atol(buffer);
				LOG<<"Got resolution "<<_resolution;
				break;
			case STATE_RESOLUTION_OFF:
				in>>buffer;

				if(stricmp(buffer,"true") == 0)
					_resolutionDisabled = true;
				else if(stricmp(buffer,"false") == 0)
					_resolutionDisabled = false;
				else 
				{
					fileIsBad = true;
					break;
				}

				break;
			default:
				LOG<<"Failed on token: "<<buffer;
				fileIsBad = true;
				break;
			}
			
			if(readObject)
			{
				//input layer
				in>>buffer;
				if(!SosUtil::is_integer(buffer))
				{
					fileIsBad = true;
					break;
				}

				object.layer = atol(buffer);

				//input value
				in>>buffer;
				if(!SosUtil::is_numeric(buffer))
				{
					fileIsBad = true;
					break;
				}

				object.value = (float)atof(buffer);

				//input point1 x
				in>>buffer;
				if(!SosUtil::is_numeric(buffer))
				{
					fileIsBad = true;
					break;
				}

				object.pt1.x = (float)atof(buffer);

				//input point1 y
				in>>buffer;
				if(!SosUtil::is_numeric(buffer))
				{
					fileIsBad = true;
					break;
				}

				object.pt1.y = (float)atof(buffer);

				//input point2 x
				in>>buffer;
				if(!SosUtil::is_numeric(buffer))
				{
					fileIsBad = true;
					break;
				}

				object.pt2.x = (float)atof(buffer);

				//input point2 y
				in>>buffer;
				if(!SosUtil::is_numeric(buffer))
				{
					fileIsBad = true;
					break;
				}

				object.pt2.y = (float)atof(buffer);

				_listObjects.push(object);
				readObject = false;

				_hasVectors = true;

				if(!_minMaxSet)
				{
					minX = SosUtil::minVal(object.pt1.x,object.pt2.x);
					maxX = SosUtil::maxVal(object.pt1.x,object.pt2.x);
					minY = SosUtil::minVal(object.pt1.y,object.pt2.y);
					maxY = SosUtil::maxVal(object.pt1.y,object.pt2.y);
					_minMaxSet = true;
				}
				else
				{
					minX = SosUtil::minVal(minX,SosUtil::minVal(object.pt1.x,object.pt2.x));
					maxX = SosUtil::maxVal(maxX,SosUtil::maxVal(object.pt1.x,object.pt2.x));
					minY = SosUtil::minVal(minY,SosUtil::minVal(object.pt1.y,object.pt2.y));
					maxY = SosUtil::maxVal(maxY,SosUtil::maxVal(object.pt1.y,object.pt2.y));
				}
			}

			if(!fileIsBad) in>>buffer;
		}		

		if(fileIsBad)
		{
			reset();

			LOG<<"Failed on token "<<buffer<<endl;
			return false;
		}
		

		LOG<<"Successfully parsed the file";
		return true;
	}

	void resetObjectIterator()
	{
		_listObjects.resetIterator();
	}

	bool getNext(LineXYLayer& obj)
	{
		return _listObjects.readNext(obj);
	}

	bool hasGridMap()
	{
		return _hasGridMap;
	}

	bool hasVectors()
	{
		return _hasVectors;
	}

	bool getDimensions(float& west, float& north, float& east, float& south)
	{
		if(!_minMaxSet)
			return false;

		west = minX;
		east = maxX;
		north = maxY;
		south = minY;
		return true;
	}

	long getResolution()
	{
		return _resolution;
	}

	bool vectorsInMM()
	{
		return _resolutionDisabled;
	}

	bool getGridMap(GridMap<float>* mapToCopyInto)
	{
		if(!_hasGridMap || mapToCopyInto == 0 || _myMap == 0)
			return false;

		mapToCopyInto->clone(_myMap);
		delete _myMap;
		_myMap = 0;

		return true;
	}

private:

	
	List<LineXYLayer> _listObjects;
	bool _hasGridMap, _hasVectors;
	bool _resolutionDisabled;

	float minX, maxX,minY,maxY;
	bool _minMaxSet;

	GridMap<float>* _myMap;
	long _resolution;
	
	DEF_LOG
};


#endif