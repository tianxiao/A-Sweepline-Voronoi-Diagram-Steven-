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

/*GridBlockQuad.h
	This file defines the class GridBlockQuad, which implements a quad tree in a 
	grid representation

 */ 
#ifndef GRIDBLOCKQUAD_H
#define GRIDBLOCKQUAD_H


#include <math.h>
#include <fstream.h>
#include <assert.h>

#define EAST 1008
#define WEST 1009
#define NORTH 1010
#define SOUTH 1011
#define ABOVE 1012
#define	BELOW 1013
#define NOTFOUND -1

#define DEFAULT_BLOCKSIZE_PSBK 64
#define DEFAULT_BLOCKHEIGHT 1
#define BUCKET_SIZE 5

#ifndef XX
#define XX 0
#endif

#ifndef YY
#define YY 1
#endif


template <class T>
class GridBlockQuad
{
  public:
	GridBlockQuad();
	GridBlockQuad(long blocksize,T defaultval,long blockheight=1);
	virtual ~GridBlockQuad();
	
	bool putVal(T value, long x,long y, long z = 0); 
	T getVal(long x, long y, long z=0);
	
	GridBlockQuad<T>* north;
	GridBlockQuad<T>* south;
	GridBlockQuad<T>* east;
	GridBlockQuad<T>* west;
	GridBlockQuad<T>* above;
	GridBlockQuad<T>* below;	
	
	long globOrigin[2];

  private:
	const long blockSize;
	const long blockHeight;

	T defaultVal;
	T* myValues;	

	GridBlockQuad<T>* topLeft;
	GridBlockQuad<T>* topRight;
	GridBlockQuad<T>* bottomLeft;
	GridBlockQuad<T>* bottomRight;
};

#endif