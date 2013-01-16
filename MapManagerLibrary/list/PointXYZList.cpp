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

#include "PointXYZList.h"

PointXYZList::PointXYZList()
{
	_myMap = new Grid3DNoFile<bool>(1000, 1,0);
}

PointXYZList::~PointXYZList()
{
	if(_myMap != 0)
		delete _myMap;
}

void PointXYZList::push(PointXYZ val)
{
	//this does a quick lookup to see if this points has already been 
	//added to the list, and removes if it has
	//This ensures that there is only one element of this value in the list
	if(_myMap->getGridRef(val.x, val.y) != 0)
	{
		PointXYZ ignore(0,0,0);
		popVal(val,ignore);
	}
	else
	{
		_myMap->updateGridRef(1,val.x, val.y);
	}
	List<PointXYZ>::push(val);
}

void PointXYZList::clear()
{
	_myMap->reset();
	List<PointXYZ>::clear();
}