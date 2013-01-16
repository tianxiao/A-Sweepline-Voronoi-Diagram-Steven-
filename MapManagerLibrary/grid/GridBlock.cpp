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
GridBlock.cpp
defines the functions of the GridBlock class
*/


#include "GridBlock.h"
#include <iostream.h>

template GridBlock<double>;
template GridBlock<int>;
template GridBlock<float>;
template GridBlock<long>;
template GridBlock<bool>;
template GridBlock<bool*>;
template GridBlock<List<LayerValue<double> >*>;
template GridBlock<List<LayerValue<float> >*>;
template GridBlock<List<LineXYLayer>*>;
template GridBlock<ListUnordered<LineXYLayer>*>;
template GridBlock<PoseRec*>;
template GridBlock<PoseRec>;
template GridBlock<PointXY*>;

LOGCODE template <class T>
LOGCODE double GridBlock<T>::cellcounter = 0;

LOGCODE template <class T>
LOGCODE double GridBlock<T>::totalPossibleCells = 0;

LOGCODE template <class T>
LOGCODE double GridBlock<T>::totalBlockSize = 0;

LOGCODE template <class T>
LOGCODE long GridBlock<T>::blockCounter = 0;


template <class T>
GridBlock<T>::GridBlock(T* defArray):blockSize(100), blockHeight(1)//, defaultVal(0)
{
	//GET_FILE_LOG
	LOGGING_OFF

	LOGENTRY("GridBlock::GridBlock() 1")

	LOGCODE blockCounter++;
	assert(XX == 0 && YY == 1);
	
	defaultArray = defArray;
	
	north=south=east=west=above=0;//below=0; 
	globOrigin[XX]=globOrigin[YY]=0;
	
	values = 0;

	values = new T**[blockHeight];
	//make each pointer in values point to another array of pointers
	//size blockSize, each of which points to an array of typ T
	//essentially this creates a three-dimensional array of type T
	for(int i = 0; i< blockHeight; i++)
	{
		values[i] = new T*[blockSize];
		
		for(int j = 0; j< blockSize; j++)
		{
			values[i][j] = 0;			
		}
	}	

	LOGCODE totalPossibleCells += blockSize * blockSize;
	LOGCODE totalBlockSize += sizeof(GridBlock<T>);
}


template <class T>
GridBlock<T>::GridBlock(long blocksize,T defaultval,long blockheight, T* defArray):blockSize(blocksize), blockHeight(blockheight)//, defaultVal(defaultval)
{
	//GET_FILE_LOG
	LOGGING_OFF
	//LOGENTRY("GridBlock::GridBlock() 2")
	LOGCODE blockCounter++;
	long counter = 0;
	
	north=south=east=west=above= 0;//below=0; 
	globOrigin[XX]=globOrigin[YY]=0;	
	
	defaultArray = defArray;	
	
	//LOG<<"Using a 3D array"<<endl;

	values = new T**[blockHeight];
	//make each pointer in values point to another array of pointers
	//size blockSize, each of which points to an array of typ T
	//essentially this creates a three-dimensional array of type T
	for(int i = 0; i< blockHeight; i++)
	{
		values[i] = new T*[blockSize];
		
		for(int j = 0; j< blockSize; j++)
		{
			values[i][j] = 0;
		}
	}

	LOGCODE totalPossibleCells += blockSize * blockSize;
	LOGCODE totalBlockSize += sizeof(GridBlock<T>);

}

template <class T>
void GridBlock<T>::init(long x, long y)
{
	values[x][y] = new T[blockSize];
	
	memcpy(values[x][y],defaultArray,blockSize * sizeof(T));
	LOGCODE cellcounter+= blockSize;

}

template <class T>
GridBlock<T>::~GridBlock()
{
	LOGCODE blockCounter--;
	
	for(int i=0; i<blockHeight; i++)
	{
		for(int j = 0; j<blockSize; j++)
		{
			if(values[i][j] != 0)
			{
				delete[] values[i][j];
				LOGCODE cellcounter-= blockSize;
			}
		}
		
		delete[] values[i];      
	}
	delete[] values;
	
//	if(deleteDefArrayOnDestroy)
//	{
//		delete[] defaultArray;
//	}
	LOGCODE totalPossibleCells -= blockSize * blockSize;
	
}

template <class T>
inline bool GridBlock<T>::putVal(T value, long x, long y, long z)
{
	if(values[z][x] == 0 )
	{
		if(value != defaultArray[0])//defaultVal)
		{
			init(z,x);
			values[z][x][y] = value;
		}
	}
	else
	{
		values[z][x][y] = value;
	}
	
	return true;
}

template <class T>
inline T GridBlock<T>::getVal(long x, long y, long z)
{
	T val;
	
	if(values[z][x] == 0 )
	{
		return defaultArray[0];
	}
	
	val = values[z][x][y];	
	
	return val;
}

template <class T>
bool GridBlock<T>::copyRow(T* arrayRef, long y,long fromX, long toX, long z)
{
	if(fromX < 0 || toX >= blockSize || fromX > toX || z > blockHeight -1 || y > blockSize)
	{
		return false;
	}
	int counter = 0;
	for(int i = fromX; i<= toX; i++)
	{
		if(values[z][i] != 0)
			arrayRef[counter++] = values[z][i][y];		
		else
			arrayRef[counter++] = defaultArray[0];
	}

	return true;
}