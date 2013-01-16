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

#ifndef GRID3D_H
#define GRID3D_H


#include "SosList.h"
#include "SosUtil.h"
#include "GridBlock.h"
#include "../logger/Logger.h"

#include <math.h>
#include <fstream.h>

#define EAST 1008
#define WEST 1009
#define NORTH 1010
#define SOUTH 1011
#define ABOVE 1012
#define	BELOW 1013
#define NOTFOUND -1

#define DEFAULT_BLOCKSIZE 100
#define DEFAULT_BLOCKHEIGHT 1

#ifndef HUGENUM
#define HUGENUM (long)1039575739848
#endif

//Interface class for copying a single row
template <class T>
class ICopyRow3D
{
	public:
		virtual bool copyRow(T* arrayRef, long y, long fromX, long toX, long z = 0) = 0;
};

template <class T>
class ICopyRow2D
{
	public:
		virtual bool copyRow(T* arrayRef, long y, long fromX, long toX) = 0;
};

template <class T>
class Grid3DNoFile: public ICopyRow3D<T>
{
	public:
		Grid3DNoFile();
		Grid3DNoFile(int blocksize, int radius, T Unknown, int blockheight = 1);
		virtual ~Grid3DNoFile();	

		inline T getGridRef(long x, long y, long z = 0);
		inline bool updateGridRef( T value, long x, long y, long z = 0);

		//copy directly another map
		void copy(Grid3DNoFile<T>* mapToCopy);

		//return the size of the map in either NORTH, SOUTH, EAST or WEST
		long getDimensions(int direction);

		void setDimensions(long west,long north,long east,long south);

		//return the size of the map in either NORTH, SOUTH, EAST or WEST
		//long getDimensions(int direction);
		long getUpdatedDimensions(int direction);

		void getAllUpdatedDimensions(long& west,long& north,long& east,long& south);

		long getMapHeight(){return getUpdatedDimensions(NORTH)-getUpdatedDimensions(SOUTH)+1;}
		long getMapWidth(){return getUpdatedDimensions(EAST)-getUpdatedDimensions(WEST)+1;}

		const T getUnknown() const{return unknown;};

		void reset();

		void crop(long west,long north,long east,long south);

		//Similar to the copy method. Whereas the copy method performs a deep copy, clone
		//performs a shallow copy.
		//Note that the map passed in to this method is destroyed.  If you don't want the
		//map to be destroyed, use the copy method
		void clone(Grid3DNoFile<T>* mapToClone);
	
		void translate(long xDist, long yDist);

		bool copyRow(T* arrayRef, long y, long fromX, long toX, long z = 0);
	protected:
		void appendBlock(GridBlock<T>* original_block, GridBlock<T>* new_block, int direction);
		int growMap(int times, int direction=0);
		GridBlock<T> * newBlock();
		
		GridBlock<T>* findBlock(long x, long y);

		void init(int blocksize, int radius, T Unknown, int blockheight);
		
		GridBlock<T>* myMap;

		GridBlock<T>* lastAccessedBlock;

		GridBlock<T>* northWestBlock;
		GridBlock<T>* southWestBlock;
		GridBlock<T>* northEastBlock;
		GridBlock<T>* southEastBlock;

		int errorVal;
		long blockSize;
		long blockHeight;
		long dimensions[6];
		long updatedDimensions[4];	

		bool isANewMap;
		T unknown;

		T* unknownArray;

		DEF_LOG
};

template <class T>
class Grid3D :public Grid3DNoFile<T>
{
	public:
		Grid3D(){};
		Grid3D(int blocksize, int radius, T Unknown, int blockheight = 1)
			:Grid3DNoFile<T>(blocksize,radius,Unknown,blockheight){};
		virtual ~Grid3D(){};		
		
		//save the map to a file
		bool save(char* filename);
		//load a map from a file
		bool load(char* filename);
};



#endif

