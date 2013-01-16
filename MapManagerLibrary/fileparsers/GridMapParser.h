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

#ifndef GRIDMAPPARSER_H
#define GRIDMAPPARSER_H

#include "../fileparsers/IncludeAll.h"
#include "../grid/GridMap.h"
#include "../sosutil/SosUtil.h"
#include "../logger/Logger.h"

class GridMapParser
{
public:
	GridMapParser()
	{
		GET_FILE_LOG
		_isCarmenMap=_isSaphira2DMap=_isBeesoftMap=_isMapViewerMap=false;
	}

	bool parseFile(char* fileName, GridMap<float>* mapToUse)
	{
		_isCarmenMap=_isSaphira2DMap=_isBeesoftMap=_isMapViewerMap=_isOldGridMap=false;

		if(mapToUse == 0)
		{
			LOG<<"mapToUse is null, so returning false";
			return false;
		}

		
		bool success = false;

		MapViewerFileParser mvmParser;
		success = mvmParser.parseMapViewerMap(fileName);

		if(success)
		{
			LOG<<"Succeeded in parsing grid map as Map Viewer Map";
			mvmParser.getGridMap(mapToUse);
			_isMapViewerMap = true;
			return true;
		}

		LOG<<"Failed to parse as MapViewer map";

		//first try to load it as a Carmen grid map
		success = CarmenTranslator::loadCarmenMap(fileName,mapToUse);

		if(success)
		{
			LOG<<"Succeeded in parsing grid map as Carmen Map";
			_isCarmenMap = true;
			return true;
		}

		LOG<<"Failed to parse as Carmen map";

		//next try to load it as a Saphira 2D laser Map
		long west,east,north,south;
		SaphiraGridParser saphParser;
		success = saphParser.parseMapFile(fileName);

		PointXYLong pt;
		
		if(success)
		{
			LOG<<"Succeeded in parsing grid map as Saphira Grid Map";
			
			mapToUse->reset();
			saphParser.resetIterator();
			while(saphParser.getNext(pt))
			{
				//Saphira grid maps only set values to 1
				mapToUse->updateGridRef(1,pt.x,pt.y);
			}
			saphParser.getDimensions(west,north,east,south);
			mapToUse->setDimensions(west,north,east,south);

			_isSaphira2DMap = true;
			return true;
		}

		LOG<<"Failed to parse as Saphira grid map";

		//try to load it as a Beesoft map
		success = CarmenTranslator::loadBeesoftMap(fileName,mapToUse);

		if(success)
		{
			LOG<<"Succeeded in parsing grid map as Beesoft Map";
			
			_isBeesoftMap = true;
			return true;
		}

		LOG<<"Failed to parse as Beesoft map";

		//finally try to load it as a MapViewer GridMap
		success = mapToUse->load(fileName);

		if(success)
		{
			LOG<<"Succeeded in parsing grid map as OLD Map Viewer Map";
			
			_isOldGridMap = true;
			return true;
		}
		LOG<<"Failed to parse the grid map";
			
		return false;
	}

	bool isCarmenMap(){return _isCarmenMap;}
	bool isSaphiraMap(){return _isSaphira2DMap;}
	bool isBeesoftMap(){return _isBeesoftMap;}
	bool isMapViewerMap(){return _isMapViewerMap;}
	bool isOldGridMap(){return _isOldGridMap;}

private:
	bool _isCarmenMap, _isSaphira2DMap,_isBeesoftMap,_isOldGridMap,_isMapViewerMap;
	DEF_LOG
};
#endif