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

#ifndef GRIDMAPLAYER_H
#define GRIDMAPLAYER_H

#include "../list/SosList.h"
#include "../sosutil/SosUtil.h"
#include "../list/SosVector.h"
#include "../grid/GridMap.h"
#include "../logger/Logger.h"

class GridMapLayer : public ICopyRow2D<float>
{
public:
	GridMapLayer(float defaultVal = 0.5);
	~GridMapLayer();

	void initFromMap(GridMap<float>* initMap, bool destroyParamMap = false);

	void disableLayerInfo();
	void enableLayerInfo(); 

	//push a single point for the given layer with the given value
	void push(long x,long y, long layer, float value);

	//remove the layer from the given grid position
	bool pop(long x, long y, long layer);

	
	void pushObject(LineXYLayer object, long resolution);
	void popObject(LineXYLayer object, long resolution);
	
	//remove all references to a given layer
	LineXYLong deleteLayer(long layer);	

	void deleteLayerPermanently(long layer);

	void deleteAllLayerInfo();

	void integrateAndDeleteLayerInfo();

	bool generateCSpace(long radius, float lowerBound, float upperBound, long squaresize = 100);

	long getWidth()
	{
		if(_baseMap == 0 || _myMap == 0)
			return 0;

		if(_baseMap->getMapWidth() > _myMap->getMapWidth())
			return _baseMap->getMapWidth() ;
		else
			return _myMap->getMapWidth();
	}

	
	long getHeight()
	{
		if(_baseMap == 0 || _myMap == 0)
			return 0;

		if(_baseMap->getMapHeight() > _myMap->getMapHeight())
			return _baseMap->getMapHeight() ;
		else
			return _myMap->getMapHeight();
	}

	LineXYLong redoLayer(long layer);

	//read the most recent layer at the given grid position returns -1 if the grid has not been set
	float read(long x, long y);

	//get the size of the map
	inline long getDimensions(int direction)
	{
		if(_baseMap == 0)
			return 0;

		switch(direction)
		{
		case NORTH:
		case EAST: return SosUtil::maxVal(_myMap->getUpdatedDimensions(direction),_baseMap->getUpdatedDimensions(direction));
		
		case WEST:
		case SOUTH:return SosUtil::minVal(_myMap->getUpdatedDimensions(direction),_baseMap->getUpdatedDimensions(direction));

		}		
		return 0;
	}

	LineXYLong getDimensions()
	{
		LineXYLong l(getDimensions(WEST),getDimensions(NORTH),getDimensions(EAST),getDimensions(SOUTH));return l;
	}
	void getDimensions(long& west,long&north,long&east,long&south);
	
	void setDimensions(long west,long north,long east,long south)
	{
		if(_baseMap != 0)
			_baseMap->setDimensions(west,north,east,south);

		if(_myMap != 0)
			_myMap->setDimensions(west,north,east,south);
	}

	void reset(bool resetBaseMap = true);

	//wipe the base map with this value
	void clearBaseMap(float value);

	void crop(long west,long north,long east,long south);

	void translate(long xDist, long yDist);

	bool copyRow(float* arrayRef, long y, long fromX, long toX);

private:

	//push a line from (x1,y1) to (x2,y2) for the given layer with the given value
	void pushLine(float x1, float y1, float x2, float y2, long layer, float value);
	
	//push an empty rectangle from (x1,y1) to (x2,y2) for the given layer with the given value
	void pushRect(long x1, long y1, long x2, long y2, long layer, float value);

	void pushRectFilled(long x1, long y1, long x2, long y2, long layer, float value);

	void popLine(float x1, float y1, float x2, float y2, long layer);

	void popRect(long x1, long y1, long x2, long y2, long layer);

	void popRectFilled(long x1, long y1, long x2, long y2, long layer);

	//push a value onto the list "_listLayerPoints" with the given layer, and the given grid coordinates
	void pushLayerVal(long layer, long x, long y, float value);

	//fill the tempMap structure with the points in a line - returns the number of points in the line
	int fillLine(float x1, float y1, float x2, float y2);

	//Looks for a reference for the given layer in "_listLayerPoints", and creates one if it doesn't exist
	List<PointXYZ>* getLayerRef(long layer);

	//Looks for a reference to the list of layers and values at a given point - creates one if it doesn't exist
	List<LayerValue<float> >* getGridRef(long x, long y,bool createNew = true);

	//At each point of the map is stored a list of LayerValue's - each of which stores the layer number 
	//and the value of the cell at that layer.  The top of the list is the most recent entry
	Grid3DNoFile<List<LayerValue<float> >*> * _myMap;

	List<LayerValue<float> >**			_oneTempRow;
	long								_oneTempRowSize;
	
	//used for temporary storage - is filled by the fillLine() method

	Vector<PointXYLong> * _lineVector;

	GridMap<float>* _baseMap;

	//stores all the points in each layer
	List<LayerValue<List<PointXYZ>*> > _listLayerPoints;

	float	_defaultValue;
	bool	_layersEnabled;

	bool _destroyMapOnInit;

	DEF_LOG
	
};







#endif