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

/*
GridMapQuad.h
specifies the GridMap class, which inherits from the Grid class
*/

#ifndef GRIDMAPQUAD_H
#define GRIDMAPQUAD_H

#include "Grid3DQuad.h"
#include "SosUtil.h"
#include "../logger/Logger.h"


//#include "pointMap.h"
#include <math.h>

#define LARGEST_VALUE_QUAD 1
#define SMALLEST_VALUE_QUAD -1


//pointMap* parseWorldFile(char* filename);


template <class T>
class GridMapQuad : public Grid3DQuad<T>
{
	public:
		GridMapQuad();
		GridMapQuad(int blocksize, int radius, T Unknown);
		GridMapQuad(GridBlockQuad<T>*mapToCopy, long* dimensions, long blocksize, T defaultValue);
		~GridMapQuad();		
		
		bool updateGridRef(T value, long x, long y){return Grid3DQuad<T>::updateGridRef(value, x,y,0);}
		T getGridRef(long x, long y){return Grid3DQuad<T>::getGridRef(x,y,0);}

		//copy() copies the map mapToCopy into this map, reducing it in size by 
		//a factor of reduceFactor 
		void copy(Grid3DQuad<T>* mapToCopy,int reduceFactor = 1,int valueToSelect = LARGEST_VALUE_QUAD);
		
		//reduceDimension() reduces the size of the map by a factor of reduceFactor
		void reduceDimension(int reduceFactor = 4, int valueToSelect = LARGEST_VALUE_QUAD);
		
		//blurs or smooths the map passed to the function
		void boxBlur(int kernelSize=3, double boxVal=1);
		void gaussBlur(int kernelSize=3);		
		
		//convert a .wld file into a gridmap
		//bool importPointMap(char* fileName, double value=1, long squareSize=100);
		
		//add a straight line from (x1,y1) to (x2, y2) to the map
		bool addLine(long x1, long y1, long x2, long y2, T value, long squareSize, bool doubleLine = true);
		
		//compare another map with this one, and give a measure of the fitness of the match
		//using image correlation methods.  This is invariant of size or orientation
		double correlateMap(GridMapQuad<T>* mapToCompare); 
		
		//compare another map with this one, and give a measure of the fitness of the match
		//using Carnegie Mellon's MATCH method. The maps must have the same origin,
		//and contain values between 0 and 1
		double scoreMap(GridMapQuad<T>* mapToCompare, bool justCompareOccAreas = false);
		

		//Grow all the occupied cells between lowerBound and upperBound by 'radius' cells
		bool growOccArea(long radius, T lowerBound, T upperBound, long squaresize = 100);

		//This method treats the map as if position (0,0) is in the top left corner
		//which is useful for windowed environments which use those corrdinates
		double getGridRefFromTopLeftView(long x, long y);		

		//This method zooms in and out of the map, allowing the user to treat
		//it as any width or height
		double getGridRefFromView(long x, long y,long viewHeight, long viewWidth);

		//This method zooms in and out of the map, allowing the user to treat
		//it as any width or height.  The four border variables specify how 
		//wide the map used is to be.  If these are not specified, the other overloaded
		//version of this method specifies them with the updated size of the map
		double getGridRefFromView(long x, long y,long viewHeight, long viewWidth
			,long westBorder, long northBorder,long eastBorder,long southBorder);

		//This method zooms in and out of the map, allowing the user to treat
		//it as any width or height.  The origin point (0,0) is treated as being at the
		//top left of the map
		double getGridRefFromTopLeftView(long x, long y,long viewHeight, long viewWidth);

		//This method zooms in and out of the map, allowing the user to treat
		//it as any width or height.  The four border variables specify how 
		//wide the map used is to be.  If these are not specified, the other overloaded
		//version of this method specifies them with the updated size of the map
		double getGridRefFromTopLeftView(long x, long y,long viewHeight, long viewWidth
			,long width, long height);

		void setTopLeftPos(long x, long y);

		bool addLineFromView(long x1, long y1, long x2, long y2, T value, long squareSize, 
			long viewHeight, long viewWidth,bool doubleLine = true);

		long getMapHeight(){return getUpdatedDimensions(NORTH)-getUpdatedDimensions(SOUTH)+1;}
		long getMapWidth(){return getUpdatedDimensions(EAST)-getUpdatedDimensions(WEST)+1;}

	private:
		
		DEF_LOG

		long topLeftX;
		long topLeftY;
		bool topLeftSet;
};


#endif
