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
GridBlock.h
Defines the GridBlock class, a child of the GridNode class
*/
#ifndef GRIDBLOCK_H
#define GRIDBLOCK_H

#include <assert.h>

#include "SosList.h"
#include "SosUtil.h"
#include "../logger/Logger.h"

#ifndef XX
#define XX 0
#endif

#ifndef YY
#define YY 1
#endif

template <class T>
class GridBlock
{
  public:
	GridBlock(T* defaultArray = 0);
	GridBlock(long blocksize, T defaultval,long blockheight=1, T* defaultArray = 0);
	virtual ~GridBlock();
	
	inline bool putVal(T value, long x,long y, long z = 0); 
	inline T getVal(long x, long y, long z=0);
	inline bool copyRow(T* arrayRef, long y, long fromX, long toX, long z);
	
	GridBlock* north;
	GridBlock* south;
	GridBlock* east;
	GridBlock* west;
	GridBlock* above;
	//GridBlock* below;
	
	long globOrigin[2];

	LOGCODE static long blockCounter; //take this out after testing
	LOGCODE static double cellcounter;
	LOGCODE static double totalPossibleCells;
	LOGCODE static double totalBlockSize;

  private:

	inline void init(long x, long y);
	const int blockSize;
	const int blockHeight;

	T*** values;

	T* defaultArray;

	DEF_LOG
};

#endif
