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

#include "GridMapLayer.h"

GridMapLayer::GridMapLayer(float defaultValue)
{
	GET_FILE_LOG
	//LOGGING_OFF
	ENTRYEXIT_LOG_OFF

	_myMap = new Grid3DNoFile<List<LayerValue<float> >*>(1000,1,0);
	
	PointXYLong discard(0,0);
	_lineVector  = new Vector<PointXYLong>(discard,100);

	_listLayerPoints.setModeOrderedDesc();

	_defaultValue = defaultValue;
	_baseMap = 0;
	_destroyMapOnInit = false;
	
	_oneTempRow = 0;
	_oneTempRowSize = 0;
	_layersEnabled = true;
}

GridMapLayer::~GridMapLayer()
{
	reset();

	if(_myMap != 0)
	{
		delete _myMap;
		_myMap = 0;
	}
		
	if(_lineVector != 0)
		delete _lineVector;

	if(_oneTempRow != 0)
		delete[] _oneTempRow;

	LOG<<"At end of GridMapLayer destructor";
}

//reset all layers, and initialise the bottom layer to the values fromt the given map
void GridMapLayer::initFromMap(GridMap<float>* initMap, bool destroyParamMap)
{
	if(initMap == 0)
		return;

	reset();

	if(_destroyMapOnInit && _baseMap != 0)
		delete _baseMap;

	
	if(!destroyParamMap)
	{
		_destroyMapOnInit = false;
		_baseMap = initMap;
	}
	else
	{
		_destroyMapOnInit = true;
		_baseMap = new GridMap<float>(100,1,0);
		_baseMap->clone(initMap);
	}

	
	long minX,maxX,minY,maxY;
	_baseMap->getAllUpdatedDimensions(minX,maxY,maxX,minY);	
	_myMap->setDimensions(minX,maxY,maxX,minY);

//	LOG<<"Copied "<<counter<<" cells into the grid layers";
}

void GridMapLayer::clearBaseMap(float value)
{
	long west  = getDimensions(WEST);
	long east  = getDimensions(EAST);
	long south = getDimensions(SOUTH);
	long north = getDimensions(NORTH);

	List<LayerValue<float> >* list = 0;
	FORX(west,east+1)
	{
		FORY(south, north+1)
		{	
			_baseMap->updateGridRef(value,x,y);
		}
	}
}


void GridMapLayer::disableLayerInfo()
{
	_layersEnabled = false;
}
void GridMapLayer::enableLayerInfo()
{
	_layersEnabled = true;

}

void GridMapLayer::reset(bool resetBaseMap)
{
	//delete all the lists - this could take a while
	long west  = _myMap->getUpdatedDimensions(WEST);
	long east  = _myMap->getUpdatedDimensions(EAST);
	long south = _myMap->getUpdatedDimensions(SOUTH);
	long north = _myMap->getUpdatedDimensions(NORTH);

	LOG<<"GridMapLayer going from ("<<west<<","<<north<<") -> ("<<east<<","<<south<<") to delete lists"<<endl;
	LOGTIME;

	List<LayerValue<float> >* list = 0;
	LayerValue<float> discard;

	for(long x = west; x <= east; x++)
	{		
		for(long y = south; y<= north; y++)
		{	
			list = _myMap->getGridRef(x,y);
			if(list != 0)
			{			
				list->clear();
				delete list;
				list = 0;
				_myMap->updateGridRef(0,x,y);
			}
		}
	}

	LOG<<"Finished deleting the lists";
	LOGTIME;

	//delete all the lists stored in the LayerPoints list
	LayerValue<List<PointXYZ>*> layerVal;
	
	_listLayerPoints.resetIterator();

	while(_listLayerPoints.popHead(layerVal))
	{
		if(layerVal.value!= 0)
			delete layerVal.value;
	}

	LOG<<"About to reset the maps";
	LOGTIME;
	_myMap->reset();
	_lineVector->clear();

	if(_destroyMapOnInit && resetBaseMap && _baseMap != 0)
	{
		delete _baseMap;
		_baseMap = 0;
	}

	LOG<<"Finished resetting the maps";

}

void GridMapLayer::push(long x,long y, long layer, float value)
{
	bool createNew = true;//!(value == _defaultValue);

	if(_layersEnabled)
	{
		List<LayerValue<float> >* list = getGridRef(x,y,createNew);

		if(list != 0)
		{
			LayerValue<float> val(layer,value);
			LayerValue<float> current(0,0);

			if( list->readHead(current))
			{
				if(current.value != value)
				{
					list->push(val);
					pushLayerVal(layer,x,y, value);
				}
			}
			else
			{
				list->push(val);
				pushLayerVal(layer,x,y, value);
			}		
		}
	}
	_baseMap->updateGridRef(value,x,y);
}

bool GridMapLayer::pop(long x, long y, long layer)
{
	List<LayerValue<float> >* list = _myMap->getGridRef(x,y);

	if(list == 0)//if there's no list, we can't pop anything off it
	{
		LOG<<"List at ("<<x<<","<<y<<") is null, so not popping anything";
		return false;
	}

	LayerValue<float> val(layer,0);

	bool retval = list->popVal(val,val); //if the value is in the list, it will be removed
	if(list->readHead(val))
	{
		_baseMap->updateGridRef(val.value,x,y);
	}
	if(list->getListSize() == 1)//0) new
	{
		delete list;
		list = 0;
		_myMap->updateGridRef(0,x,y);
	}

	return retval;
}

float GridMapLayer::read(long x, long y)
{
	return _baseMap->getGridRef(x,y);
}

void GridMapLayer::pushLine(float x1, float y1, float x2, float y2, long layer, float value)
{
	LOGENTRY("pushLine")
	int numCells = fillLine(x1,y1,x2,y2);

	LOG<<"pushLine(): after fillLine("<<x1<<","<<y1<<") -> ("<<x2<<","<<y2<<") and numCells = "<<numCells<<endl;
	long x = 0, y = 0;
	List<LayerValue<float> >* list = 0;

	//LOG<<"After creating a List<LayerValue<float> >*"<<endl;
	LayerValue<float> val;	

	PointXY pt(0,0);
	PointXYLong ptLong(0,0);
	if(_layersEnabled)
	{
		LOG<<"LayersEnabled, so pushing the line onto the grid list";
		for(int i = 0; i<numCells; i++)
		{
			ptLong = _lineVector->get(i);
			x = ptLong.x;
			y = ptLong.y;

			list = getGridRef(x,y);	
			
			val.layerNumber = layer;
			val.value = value;
			list->push(val);
			_baseMap->updateGridRef(value,x,y);//new
		}
	}
	else
	{
		for(int i = 0; i<numCells; i++)
		{
			ptLong = _lineVector->get(i);
			x = ptLong.x;
			y = ptLong.y;

			_baseMap->updateGridRef(value,x,y);//new
		}
	}


	LOGEXIT("pushLine")
}

//push a filled rectangle from (x1,y1) to (x2,y2) with the given value belonging to the given layer
void GridMapLayer::pushRect(long x1, long y1, long x2, long y2, long layer, float value)
{
	long x = 0, y = 0;
	List<LayerValue<float> >* list = 0;
	LayerValue<float> val(0,0);
	PointXY pt(0,0);

	LOG<<"pushRect got ("<<x1<<","<<y1<<") -> ("<<x2<<","<<y2<<")";

	SosUtil::ensureSmaller(x1,x2);
	SosUtil::ensureSmaller(y1,y2);

	val.layerNumber = layer;
	val.value = value;

	long x1L = (x1 < 0 && float((long)x1) != x1)? long(x1 - 1):long(x1);
	long y1L = (y1 < 0 && float((long)y1) != y1)? long(y1 - 1):(long)y1;
	long x2L = (x2 < 0 && float((long)x2) != x2)? long(x2 - 1):(long)x2;
	long y2L = (y2 < 0 && float((long)y2) != y2)? long(y2 - 1):(long)y2;
	LOG<<"pushRect changed to ("<<x1L<<","<<y1L<<") -> ("<<x2L<<","<<y2L<<")";

	if(_layersEnabled)
	{
		for(x= x1L; x<= x2L; x++)
		{			
			list = getGridRef(x,y1L);//get the list of set points at this grid position
			list->push(val);	
			_baseMap->updateGridRef(value,x,y1L);

			list = getGridRef(x,y2L);
			list->push(val);
			_baseMap->updateGridRef(value,x,y2L);
		}

		for(y = y1L + 1; y < y2L; y++)
		{
			list = getGridRef(x1L,y);
			list->push(val);
			_baseMap->updateGridRef(value,x1L,y);
			list = getGridRef(x2L,y);
			list->push(val);
			_baseMap->updateGridRef(value,x2L,y);
		}
	}
	else
	{
		for(x= x1L; x<= x2L; x++)
		{			
			_baseMap->updateGridRef(value,x,y1L);
			_baseMap->updateGridRef(value,x,y2L);
		}

		for(y = y1L + 1; y < y2L; y++)
		{
			_baseMap->updateGridRef(value,x1L,y);
			_baseMap->updateGridRef(value,x2L,y);
		}
	}
}

void GridMapLayer::pushRectFilled(long x1, long y1, long x2, long y2, long layer, float value)
{
	List<LayerValue<float> >* list = 0;
	LayerValue<float> val(0,0);
	val.layerNumber = layer;
	val.value = value;

	LOG<<"pushRectFilled got ("<<x1<<","<<y1<<") -> ("<<x2<<","<<y2<<")";

	SosUtil::ensureSmaller(x1,x2);
	SosUtil::ensureSmaller(y1,y2);

	if(_layersEnabled)
	{
		for(long x= x1; x<= x2; x++)
		{
			for(long y = y1; y <= y2; y++)
			{
				list = getGridRef(x,y);
				list->push(val);
				_baseMap->updateGridRef(value,x,y);
			}
		}
	}
	else
	{
		for(long x= x1; x<= x2; x++)
		{
			for(long y = y1; y <= y2; y++)
			{
				_baseMap->updateGridRef(value,x,y);
			}
		}
	}
}

//push a point reference onto the list of points belonging to a given layer
void GridMapLayer::pushLayerVal(long layer, long x, long y, float value)
{
	List<PointXYZ>* layerList = getLayerRef(layer);	

	PointXYZ pt(x,y,value);
	layerList->push(pt);

	LOG<<"pushLayerVal("<<layer<<","<<x<<","<<y<<")";
}

List<LayerValue<float> >* GridMapLayer::getGridRef(long x, long y, bool createNew)
{
	LOGENTRY("getGridRef")
	
	List<LayerValue<float> >* list = _myMap->getGridRef(x,y);

//	LOG<<"getGridRef(): Got list = "<<list<<" from the map"<<endl;
	if(list == 0 && createNew)//if this point has not previously been set, then create a list
	{
	//	LOG<<"getGridRef(): Creating a new list"<<endl;
		list = new List<LayerValue<float> >;
		list->setModeStack();		
		_myMap->updateGridRef(list,x,y);
		
		LayerValue<float> val(0,_baseMap->getGridRef(x,y));//new
		list->push(val);//new

		//LOG<<"getGridRef(): updated the map with the new list, and pushed the value "<<_baseMap->getGridRef(x,y);
	}

	LOGEXIT("getGridRef")
	return list;

}

//find the list of points in a layer - if the list doesn't exist, create an empty one and return a reference to it
List<PointXYZ>* GridMapLayer::getLayerRef(long layer)
{
	LOGENTRY("getLayerRef");
	LayerValue<List<PointXYZ>*> layerList(layer,0,false);

	bool layerExists = _listLayerPoints.readVal(layerList,layerList);
	
	if(!layerExists)//if the layer is not already in the list, then create it
	{
		LOG<<"getLayerRef(): creating the list for layer "<<layer<<endl;
		layerList.layerNumber = layer;
		layerList.value = new List<PointXYZ>;
		layerList.value->setModeStack();

		_listLayerPoints.push(layerList);
		LOG<<"After pushing on to the layer list"<<endl;
	}
	LOG<<"returning "<<layerList.value<<endl;

	LOGEXIT("getLayerRef");
	return layerList.value;
}


LineXYLong GridMapLayer::redoLayer(long layer)
{
	LayerValue<List<PointXYZ>*> layerList(layer,0,false);
	List<LayerValue<float> >* gridList = 0;//this is a list of all layers and values at a (x,y) coordinate
	LayerValue<float> layerVal(layer,0);//this is the value that the gridList will have pushed on to it

	PointXYZ pt(0,0,0);

	LineXYLong ret;

	LOG<<"redoLayer()";

	long minX= 0, maxX = 0, minY = 0, maxY = 0;
	bool layerExists = _listLayerPoints.readVal(layerList,layerList);

	if(layerExists)
	{
		LOG<<"The layer "<<layer<<" does exist";
		List<PointXYZ> * list = layerList.value;
		if(list->readHead(pt))
		{
			minX = maxX = pt.x;
			minY = maxY = pt.y;
		}

		list->resetIterator();	

		while(list->readNext(pt))
		{
			//get the list of layers and points at this grid position
			gridList = getGridRef(pt.x,pt.y);

			//if layers have been set at this grid position, remove all references to the layer to be deleted
			if(gridList != 0)
			{	
				layerVal.value = pt.value;
				gridList->push(layerVal);
				minX = SosUtil::minVal(minX,pt.x);
				maxX = SosUtil::maxVal(maxX,pt.x);
				minY = SosUtil::minVal(minY,pt.y);
				maxY = SosUtil::maxVal(maxY,pt.y);				
			}
			_baseMap->updateGridRef(layerVal.value,pt.x,pt.y);
		}
	}
	else
	{
		LOG<<"The layer "<<layer<<" does NOT exist";
	}
	ret.pt1.x = minX;
	ret.pt1.y = maxY;
	ret.pt2.x = maxX;
	ret.pt2.y = minY;
	return ret;

}


LineXYLong GridMapLayer::deleteLayer(long layer)
{
	LayerValue<List<PointXYZ>*> layerList(layer,0,false);
	List<LayerValue<float> >* gridList = 0;//this is a list of all layers and values at a (x,y) coordinate
	LayerValue<float> layerVal(layer,0);//this is the key that the gridList will be searched with
	LayerValue<float> tempLayerVal(0,0);//this is a throwaway variable that the popVal() method writes to
	
	long minX= 0, maxX = 0, minY = 0, maxY = 0;
	//get the list of all the points in this layer
	bool layerExists = _listLayerPoints.readVal(layerList,layerList);

	if(layerExists)
	{	
		List<PointXYZ> * list = layerList.value;
		PointXYZ pt(0,0,0);		

		if(list->readHead(pt))
		{
			minX = maxX = pt.x;
			minY = maxY = pt.y;

		//	LOG<<"initialised minX and maxX to "<<pt.x;
		//	LOG<<"initialised minY and maxY to "<<pt.y;
		}

		list->resetIterator();	

		while(list->readNext(pt))
		{
			//get the list of layers and points at this grid position
			gridList = _myMap->getGridRef(pt.x,pt.y);
		//	LOG<<"read point ("<<pt.x<<","<<pt.y<<")";

			//if layers have been set at this grid position, remove all references to the layer to be deleted
			if(gridList != 0)
			{	
				LOG<<"The list "<<gridList<<" is not null, so popping a value off it"<<endl;
				gridList->popVal(layerVal,tempLayerVal);//remove the reference to this layer at this grid position
				minX = SosUtil::minVal(minX,pt.x);
				maxX = SosUtil::maxVal(maxX,pt.x);
				minY = SosUtil::minVal(minY,pt.y);
				maxY = SosUtil::maxVal(maxY,pt.y);

				if(gridList->getListSize() == 0)
				{					
					_myMap->updateGridRef(0,pt.x,pt.y);
					LOG<<"Deleting list "<<gridList<<" at pos ("<<pt.x<<","<<pt.y<<")"<<endl;
					delete gridList;
					gridList = 0;
				}
				else
				{
					gridList->readHead(tempLayerVal);//new
					
					LOG<<"Did NOT delete the list because list size = "<<gridList->getListSize();
				}
				_baseMap->updateGridRef(tempLayerVal.value,pt.x,pt.y);//new

			//	LOG<<"minX = "<<minX<<", maxX = "<<maxX<<", minY = "<<minY<<", maxY = "<<maxY;
			}
		}

//		delete list;
	}

	LineXYLong ret(minX, minY, maxX,maxY);

	LOG<<"At end of GridMapLayer::deleteLayer("<<layer<<")";
	return ret;
}

void GridMapLayer::integrateAndDeleteLayerInfo()
{
	if(_baseMap == 0)
	{
		deleteAllLayerInfo();
		return;
	}

	List<LayerValue<float> >* gridList = 0;
	long north = _myMap->getUpdatedDimensions(NORTH);
	long south = _myMap->getUpdatedDimensions(SOUTH);
	long east = _myMap->getUpdatedDimensions(EAST);
	long west = _myMap->getUpdatedDimensions(WEST);
	
	LayerValue<float> val;
	List<LayerValue<float> > ** listArr = new List<LayerValue<float> >*[east - west +1];

	for(long y = south; y <= north; y++)
	{
		_myMap->copyRow(listArr,y,west,east);
		for(long x = west; x <= east; x++)
		{
			//gridList = _myMap->getGridRef(x,y);
			gridList = listArr[x - west];
			if(gridList != 0)
			{	
				if(gridList->popHead(val))
				{
				//	_baseMap->updateGridRef(val.value,x,y);		//new			
				
					while(gridList->getListSize() > 1)
					{
						gridList->popTail(val);
					}
				}

				delete gridList;
				_myMap->updateGridRef(0,x,y);
			}
		}
	}
	delete [] listArr;

}

void GridMapLayer::deleteAllLayerInfo()
{
	List<LayerValue<float> >* gridList = 0;
	long north = _myMap->getUpdatedDimensions(NORTH);
	long south = _myMap->getUpdatedDimensions(SOUTH);
	long east = _myMap->getUpdatedDimensions(EAST);
	long west = _myMap->getUpdatedDimensions(WEST);
	
	LayerValue<float> val;

	
	for(long x = west; x <= east; x++)
	{
		for(long y = south; y <= north; y++)
		{
			gridList = _myMap->getGridRef(x,y);
			if(gridList != 0)
			{	
				LOG<<"GridList size = "<<gridList->getListSize();
				while(gridList->getListSize() > 1)
				{
					gridList->popHead(val);
					LOG<<"Popped the value "<<val.value;
				}
				LOG<<"Updated the base map with value "<<val.value;
				//_baseMap->updateGridRef(val.value,x,y);
				//delete gridList;
				//_myMap->updateGridRef(0,x,y);				
			}
		}
	}

	LOGCODE _baseMap->save("c:\\temp\\testListDel.map");

	_listLayerPoints.clear();
}

void GridMapLayer::deleteLayerPermanently(long layer)
{
	LayerValue<List<PointXYZ>*> layerList(layer,0,false);
	if(_listLayerPoints.popVal(layerList,layerList))
	{
		delete layerList.value;
	}
}

int GridMapLayer::fillLine(float x1_, float y1_, float x2_, float y2_)
{
	long temp = 0;
    double slope = 0; //this is the 'm' in the line formula y = mx+c
    double yIntercept = 0; //this is the 'c' in the line formula y = mx+c
    double x=0,y=0;

	int counter = 0;
	PointXYLong ptLong(0,0);
    bool noSlope = false;

	long tempX = 0, tempY = 0;


	LOG<<"1.fillLine("<<x1_<<","<<y1_<<","<<x2_<<","<<y2_<<")";

	double x1 = x1_ , x2 = x2_ ,y1 = y1_ , y2 = y2_ ;

	LOG<<"2.fillLine("<<x1<<","<<y1<<","<<x2<<","<<y2<<")";
	
    //if the line doesn't deviate from a single line of squares, then there's no need to calculate the slope
    //This also prevents division by zero
    if(y2 != y1 && x1 != x2)
    {
		slope = double(y2 - y1)/double(x2 - x1);
		noSlope = false;			
    }
    else
    {
		noSlope = true;
    }    
	
    yIntercept = y1 - (slope * x1); //if y=mx+c, then c = y - mx
	
	
    if(fabs(y2 - y1) >= fabs(x2 - x1))
    {   //if the distance between the two y's is greater than or equal to the distance
		//between the two x's, we process the line in the y direction.
		//otherwise we process it in the x direction
		
		if(y2 < y1)//if y2 is below y1, swap the 2 points
		{
			SosUtil::swap(y2,y1);
			SosUtil::swap(x2,x1);
		}
		
		//go from y1 to the grid cell containing y2, which is therefore rounded up to the value at the top of the cell
		if(!noSlope)
		{		
			LOG<<"Line drawer 1";
			for(y=y1; (long)y <= (long)y2  ; y += 1)
			{
				if(y > y2)
					y = y2;

				x = (y - yIntercept)/slope ; //if y = mx +c, then x = (y - c)/m
				//LOG<<"y = "<<y<<", x = "<<(y - yIntercept)/slope<<" = "<<x<<endl;

				tempX = (x < 0) ? long(x-1):long(x);
				tempY = (y < 0)  ? long(y-1):long(y);					
				
				ptLong.x = tempX;
				ptLong.y = tempY;
				_lineVector->put(counter,ptLong);
				LOG<<"Set ("<<tempX<<","<<tempY<<")";
				counter++;	
			}		    
			
		}
		else
		{		
			LOG<<"Line drawer 2";
			x = x1;
			for(y=y1; (long)y <= (long)y2 ; y += 1)
			{
				tempX = (x < 0) ? long(x-1):long(x);
				tempY = (y < 0) ? long(y-1):long(y);					
				
				ptLong.x = tempX;
				ptLong.y = tempY;
				_lineVector->put(counter,ptLong);
				LOG<<"Set ("<<tempX<<","<<tempY<<")";
				counter++;
			}				
		}
    } 
    else //step in the x direction
    {	
		if(x2 < x1)//if x2 to the left of x1, swap the 2 points
		{
			SosUtil::swap(y2,y1);
			SosUtil::swap(x2,x1);
		}
		
		if(!noSlope)
		{		
			LOG<<"Line drawer 3";
			//go from x1 to the grid cell containing x2, which is therefore rounded up to the value at the right of the cell
			for(x=x1; (long)x <= (long)x2 ; x += 1)
			{
				if(x > x2)
					x = x2;

				y = (slope * x) + yIntercept ; // y = mx +c
				
				tempX = (x < 0) ? long(x-1):long(x);
				tempY = (y < 0)  ? long(y-1):long(y);			
				
				ptLong.x = tempX;
				ptLong.y = tempY;
				_lineVector->put(counter,ptLong);
				counter++;
			}
			
			
		}
		else
		{
			LOG<<"Line drawer 4";
			y = y1;
			//go from x1 to the cell containing x2,which is therefore rounded up to the value at the right of the cell
			for(x=x1; (long)x <= (long)x2 ; x += 1)
			{
				tempX = (x < 0) ? long(x-1):long(x);
				tempY = (y < 0)  ? long(y-1):long(y);			
				
				ptLong.x = tempX;
				ptLong.y = tempY;
				_lineVector->put(counter,ptLong);
				counter++;
			}
		}
    }

	LOG<<"At end of fillLine, counter = "<<counter<<" and vector size = "<<_lineVector->size();

	return counter;
}

void GridMapLayer::pushObject(LineXYLayer object, long resolution)
{	
	object.pt1.x /= resolution;
	object.pt1.y /= resolution;
	object.pt2.x /= resolution;
	object.pt2.y /= resolution;

	
	switch(object.type)
	{
	case OBJECT_TYPE_LINE:
		pushLine(object.pt1.x,object.pt1.y,object.pt2.x,object.pt2.y,object.layer,object.value);
		break;
	case OBJECT_TYPE_RECTANGLE:
		object.pt1.x = (object.pt1.x < 0) ? object.pt1.x - 1 : object.pt1.x;
		object.pt1.y = (object.pt1.y < 0) ? object.pt1.y - 1 : object.pt1.y;
		object.pt2.x = (object.pt2.x < 0) ? object.pt2.x - 1 : object.pt2.x;
		object.pt2.y = (object.pt2.y < 0) ? object.pt2.y - 1 : object.pt2.y;
		pushRect((long)object.pt1.x,(long)object.pt1.y,(long)object.pt2.x,(long)object.pt2.y,object.layer,object.value);
		break;
	case OBJECT_TYPE_RECTANGLE_FILLED:
		object.pt1.x = (object.pt1.x < 0) ? object.pt1.x - 1 : object.pt1.x;
		object.pt1.y = (object.pt1.y < 0) ? object.pt1.y - 1 : object.pt1.y;
		object.pt2.x = (object.pt2.x < 0) ? object.pt2.x - 1 : object.pt2.x;
		object.pt2.y = (object.pt2.y < 0) ? object.pt2.y - 1 : object.pt2.y;
		pushRectFilled((long)object.pt1.x,(long)object.pt1.y,(long)object.pt2.x,(long)object.pt2.y,object.layer,object.value);
		break;
	}
}

void GridMapLayer::popObject(LineXYLayer object, long resolution)
{
	//LOG<<"popObject: "<<object<<", resolution = "<<resolution;
	object.pt1.x /= resolution;
	object.pt1.y /= resolution;
	object.pt2.x /= resolution;
	object.pt2.y /= resolution;

	switch(object.type)
	{
	case OBJECT_TYPE_LINE:
		popLine(object.pt1.x,object.pt1.y,object.pt2.x,object.pt2.y,object.layer);
		break;
	case OBJECT_TYPE_RECTANGLE:
		object.pt1.x = (object.pt1.x < 0) ? object.pt1.x - 1 : object.pt1.x;
		object.pt1.y = (object.pt1.y < 0) ? object.pt1.y - 1 : object.pt1.y;
		object.pt2.x = (object.pt2.x < 0) ? object.pt2.x - 1 : object.pt2.x;
		object.pt2.y = (object.pt2.y < 0) ? object.pt2.y - 1 : object.pt2.y;
		popRect((long)object.pt1.x,(long)object.pt1.y,(long)object.pt2.x,(long)object.pt2.y,object.layer);
		break;
	case OBJECT_TYPE_RECTANGLE_FILLED:
		object.pt1.x = (object.pt1.x < 0) ? object.pt1.x - 1 : object.pt1.x;
		object.pt1.y = (object.pt1.y < 0) ? object.pt1.y - 1 : object.pt1.y;
		object.pt2.x = (object.pt2.x < 0) ? object.pt2.x - 1 : object.pt2.x;
		object.pt2.y = (object.pt2.y < 0) ? object.pt2.y - 1 : object.pt2.y;
		popRectFilled((long)object.pt1.x,(long)object.pt1.y,(long)object.pt2.x,(long)object.pt2.y,object.layer);
		break;
	}
}


void GridMapLayer::popLine(float x1, float y1, float x2, float y2, long layer)
{
	LOGENTRY("popLine")
	int numCells = fillLine(x1,y1,x2,y2);

	LOG<<"popLine("<<x1<<","<<y1<<","<<x2<<","<<y2<<"): got "<<numCells<<" in the line";

	long x = 0, y = 0;
	List<LayerValue<float> >* list = 0;
	bool retval = 0;

	LayerValue<float> val;

	PointXY pt(0,0);
	PointXYLong ptLong(0,0);
	for(int i = 0; i<numCells; i++)
	{		
		ptLong = _lineVector->get(i);
		x = ptLong.x;
		y = ptLong.y;

		retval = pop(x,y,layer);
	//	LOG<<"("<<x<<","<<y<<") in layer "<<layer<<" did "<<(retval ? "NOT":"")<<" pop successfully";

	}

	LOGEXIT("popLine")
}

void GridMapLayer::popRect(long x1, long y1, long x2, long y2, long layer)
{
	LOGENTRY("popRect")

	SosUtil::ensureSmaller(x1,x2);
	SosUtil::ensureSmaller(y1,y2);

	for(long x= x1; x<= x2; x++)
	{
		pop(x,y1,layer);
		pop(x,y2,layer);
	}	

	for(long y = y1+1; y < y2; y++)
	{
		pop(x1,y,layer);
		pop(x2,y,layer);
	}

		
	LOGEXIT("popRect")
}

void GridMapLayer::popRectFilled(long x1, long y1, long x2, long y2, long layer)
{
	LOGENTRY("popRect")

	List<LayerValue<float> >* list = 0;

	SosUtil::ensureSmaller(x1,x2);
	SosUtil::ensureSmaller(y1,y2);
	LayerValue<float> val(layer,0);

	for(long x= x1; x<= x2; x++)
	{
		for(long y = y1; y <= y2; y++)
		{
			pop(x,y, layer);			
		}
	}	
		
	LOGEXIT("popRect")
}

void GridMapLayer::crop(long west,long north,long east,long south)
{
	SosUtil::ensureSmaller(west,east);
	SosUtil::ensureSmaller(south,north);

	LOG<<"GridMapLayer cropping ("<<west<<","<<north<<")->("<<east<<","<<south<<")";

	long minX,maxX,minY,maxY;
	getDimensions(minX,maxY,maxX,minY);

	SosUtil::ensureSmaller(minX,maxX);
	SosUtil::ensureSmaller(minY,maxY);

	minX = SosUtil::minVal(minX,west);
	maxX = SosUtil::maxVal(maxX,east);
	minY = SosUtil::minVal(minY,south);
	maxY = SosUtil::maxVal(maxY,north);

	List<LayerValue<float> >*list = 0;

	LOG<<"Going to iterate from ("<<minX<<","<<maxY<<") ->("<<maxX<<","<<minY<<")";

	for(long x = minX; x<= maxX; x++)
	{
		for(long y = minY; y<= maxY; y++)
		{
			if(x < west || x > east || y > north || y < south)
			{
				list = _myMap->getGridRef(x,y);
				if(list != 0)
				{
					delete list;
					list = 0;
					_myMap->updateGridRef(0,x,y);
				}
			}
		}
	}
	
	LOG<<"about to crop the base map";
	_baseMap->crop(west,north,east,south);
	LOG<<"Finished cropping the base map";
	_baseMap->setDimensions(west,north,east,south);

	west = SosUtil::maxVal(west,_myMap->getUpdatedDimensions(WEST));	
	east = SosUtil::minVal(east,_myMap->getUpdatedDimensions(EAST));	
	south = SosUtil::maxVal(south,_myMap->getUpdatedDimensions(SOUTH));
	north = SosUtil::minVal(north,_myMap->getUpdatedDimensions(NORTH));

	_myMap->crop(west,north,east,south);

	LOG<<"Finished cropping _myMap";
	deleteAllLayerInfo();//since this cannot be undone, no point keeping layer info
	
	_myMap->setDimensions(west,north,east,south);

}

void GridMapLayer::translate(long xDist, long yDist)
{
	if(_baseMap != 0)
	{
		_baseMap->translate(xDist,yDist);
	}
	if(_myMap != 0)
	{
		_myMap->translate(xDist,yDist);
	}
	deleteAllLayerInfo();//changed the whole map - no point keeping undo info

}

void GridMapLayer::getDimensions(long& west,long&north,long&east,long&south)
{
	west = getDimensions(WEST);
	north = getDimensions(NORTH);
	south = getDimensions(SOUTH);
	east = getDimensions(EAST);	
}


bool GridMapLayer::copyRow(float* arrayRef, long y, long fromX, long toX)
{
	return _baseMap->copyRow(arrayRef,y,fromX,toX);	
}

bool GridMapLayer::generateCSpace(long radius, float lowerBound, 
								  float upperBound, long squaresize)
{
	if(_baseMap == 0)
		return false;

	long west=0,east=0,south=0,north=0;
	getDimensions(west,north,east,south);
	List<LayerValue<float> >* gridList = 0;
	LayerValue<float> val;


	integrateAndDeleteLayerInfo();
	
	_baseMap->setDimensions(west,north,east,south);
	_baseMap->growOccArea(radius,lowerBound,upperBound,squaresize);
/*
	west = SosUtil::minVal(west,_baseMap->getUpdatedDimensions(WEST));
	east = SosUtil::maxVal(east,_baseMap->getUpdatedDimensions(EAST));
	north =SosUtil::maxVal(north,_baseMap->getUpdatedDimensions(NORTH));
	south = SosUtil::minVal(south,_baseMap->getUpdatedDimensions(SOUTH));
	_baseMap->setDimensions(west,north,east,south);
	_myMap->setDimensions(west,north,east,south);
*/
	
	return true;
}