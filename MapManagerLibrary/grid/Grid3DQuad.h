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
Grid3D.h
specifies the Grid class, the parent of the GridMap and GridMap3d classes
*/

#ifndef GRID3DQUAD_H
#define GRID3DQUAD_H

#include "GridBlockQuad.h"

#include <math.h>
#include <fstream.h>

#define EAST 1008
#define WEST 1009
#define NORTH 1010
#define SOUTH 1011
#define ABOVE 1012
#define	BELOW 1013
#define NOTFOUND -1

#define DEFAULT_BLOCKSIZE_QUAD 64
#define DEFAULT_BLOCKHEIGHT_QUAD 1

#ifndef HUGENUM
#define HUGENUM (long)1039575739848
#endif


template <class T>
class Grid3DQuad
{
	public:
		Grid3DQuad();
		Grid3DQuad(int blocksize, int radius, T Unknown, int blockheight = 1);
		virtual ~Grid3DQuad();	

		T getGridRef(long x, long y, long z = 0);
		bool updateGridRef( T value, long x, long y, long z = 0);

		//copy directly another map
		void copy(Grid3DQuad<T>* mapToCopy);

		//save the map to a file
		bool save(char* filename);
		//load a map from a file
		bool load(char* filename);

		//return the size of the map in either NORTH, SOUTH, EAST or WEST
		long getDimensions(int direction);

		//return the size of the map in either NORTH, SOUTH, EAST or WEST
		//long getDimensions(int direction);
		long getUpdatedDimensions(int direction);

		const T getUnknown() const{return unknown;};

		void reset();
	
	protected:
		void appendBlock(GridBlockQuad<T>* original_block, GridBlockQuad<T>* new_block, int direction);
		int growMap(int times, int direction=0);
		GridBlockQuad<T> * newBlock();
		
		GridBlockQuad<T>* findBlock(long x, long y);
		
		GridBlockQuad<T>* myMap;

		GridBlockQuad<T>* lastAccessedBlock;

		int errorVal;
		long blockSize;
		long blockHeight;
		long dimensions[6];
		long updatedDimensions[4];	

		bool isANewMap;
		T unknown;

};



#endif

