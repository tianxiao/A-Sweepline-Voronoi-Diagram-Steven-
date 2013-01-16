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

#ifndef Grid3DNoFileCPP
#define Grid3DNoFileCPP

#include "Grid3D.h"
#include <iostream.h>
#include <stdlib.h>
#include <string.h>

template Grid3DNoFile<double>;
template Grid3DNoFile<int>;
template Grid3DNoFile<float>;
template Grid3DNoFile<long>;
template Grid3DNoFile<bool>;
template Grid3DNoFile<bool*>;
template Grid3DNoFile<List<LayerValue<float> >*>;
template Grid3DNoFile<List<LineXYLayer>*>;
template Grid3DNoFile<ListUnordered<LineXYLayer>*>;
template Grid3DNoFile<PoseRec*>;
template Grid3DNoFile<PoseRec>;
template Grid3DNoFile<PointXY*>;

template Grid3D<double>;
template Grid3D<int>;
template Grid3D<float>;
template Grid3D<long>;

template <class T>
Grid3DNoFile<T>::Grid3DNoFile()
{
//	GET_FILE_LOG
	LOGGING_OFF

	init(DEFAULT_BLOCKSIZE,1,0,DEFAULT_BLOCKHEIGHT);
	
	
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------

template <class T>
Grid3DNoFile<T>::Grid3DNoFile(int blocksize, int radius, T Unknown, int blockheight)
{
	//GET_FILE_LOG
	LOGGING_OFF

	init(blocksize,radius,Unknown,blockheight);

}
template <class T>
void Grid3DNoFile<T>::init(int blocksize, int radius, T Unknown, int blockheight)
{
	unknown = Unknown;
    blockSize = blocksize;
    blockHeight = blockheight;

	assert(blockHeight > 0);

	unknownArray = new T[blockSize];

	int x = 0;
	for(x = 0; x<blockSize; x++)
	{
		unknownArray[x] = unknown;
	}
	
	for(x = 0; x < 4; x++)
	{
		updatedDimensions[x] = 0;
		dimensions[x] = 0;
	}
    myMap = newBlock();	
	northWestBlock = southWestBlock = northEastBlock = southEastBlock = myMap;
	reset();

    //make the map have a radius of 'radius + 1' number of blocks
    if(radius > 0) 
		growMap(radius);
	    
	
    errorVal = 0;
	
	isANewMap = true;//this is used with the updatedDimensions. if it is true, then the first
	//cell to be updated becomes the newly updated DImensions for NORTH, SOUTH, EAST and WEST


	//when a user accesses a grid cell, it is very likely that it will be close to the cell they 
	//last accessed, so to speed up the search, instead of searching from the origin (0,0) every time
	//we search from the last position accessed
	lastAccessedBlock = myMap;


}

//---------------------------------------------------------------------
//---------------------------------------------------------------------

template <class T>
Grid3DNoFile<T>::~Grid3DNoFile()
{
	LOG<<"At start of ~Grid3DNoFile()";
    if(myMap != 0)
    {
		reset();
		delete myMap;
    }

	if(unknownArray != 0)
		delete[] unknownArray;
	
    myMap = 0;
	LOG<<"At end of ~Grid3DNoFile()";
}


//---------------------------------------------------------------------
//---------------------------------------------------------------------

template <class T>
inline T Grid3DNoFile<T>::getGridRef(long x, long y, long z)
{
    errorVal = 0;
    T retval;
    GridBlock<T> *current;
	
    if(z > blockHeight - 1 || y > dimensions[NORTH % 6]-1 || y < dimensions[SOUTH % 6]-1
		|| x > dimensions[EAST % 6]-1 || x < dimensions[WEST % 6]-1)
    {
		return unknown;
    }
	
    current = findBlock(x,y);
	
    if(current == 0)//
    {
		errorVal = NOTFOUND;
		return unknown;
    }
    else
    {
		retval = current->getVal(int(fabs(current->globOrigin[XX] - x)),int(fabs(current->globOrigin[YY] - y)),z);
		return retval;
    }	
	
	
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------


template <class T>
inline bool Grid3DNoFile<T>::updateGridRef( T newValue, long x, long y, long z )
{
    int noOfBlocks = 0;	
    
    GridBlock<T>* current = 0;
	
    if(z > blockHeight - 1)
    {
		return false;
    }
	//northWestBlock = southWestBlock = northEastBlock = southEastBlock
    current = findBlock(x,y); 	
	
    while(current == 0)
    {		
		current = myMap;
		switch(errorVal) //decide which direction to grow the map in
		{	
		case NORTH: //find the northernmost block of the map and calculate how many 
			//blocks are needed to reach the new one		

			current = northWestBlock;
			
			noOfBlocks = int(fabs(y - current->globOrigin[YY])/ blockSize);
			
			if(noOfBlocks == 0) //if noOfBlocks is rounded down because of its conversion to integer
				noOfBlocks++;   //we need to increment it
						
			growMap(noOfBlocks, NORTH);
			break;
			
		case SOUTH: //find the southernmost block of the map and calculate how many 
			//blocks are needed to reach the new one
	
			current = southWestBlock;

			noOfBlocks = int(fabs(y - current->globOrigin[YY])/ blockSize);
			
			if(noOfBlocks == 0) 
				noOfBlocks++;
			
			growMap(noOfBlocks, SOUTH);
			break;
			
		case EAST: //find the easternmost block of the map and calculate how many 
			//blocks are needed to reach the new one
		
			current = southEastBlock;
			
			noOfBlocks = int(fabs(x - current->globOrigin[XX])/ blockSize);
			
			if(noOfBlocks == 0) 
				noOfBlocks++;
			
			growMap(noOfBlocks, EAST);
			break;
			
		case WEST: //find the westernmost block of the map and calculate how many 
			//blocks are needed to reach the new one
	
			current = northWestBlock;
			
			noOfBlocks = int(fabs(x - current->globOrigin[XX])/ blockSize);
			
			if(noOfBlocks == 0) 
				noOfBlocks++;
			
			growMap(noOfBlocks, WEST);
			break;
			
		default: 
			
			growMap(1); //if the direction has not been reported, just grown the whole map
			//in every direction by one block, and try again
			break;
		}
		
		current = findBlock(x,y);
		
    }//end while(current == unknown)
	
    //put the new (x,y) value into the grid reference
    current->putVal(newValue,int(fabs(current->globOrigin[XX] - x)),int(fabs(current->globOrigin[YY] - y)), z); 

	if(isANewMap && newValue != unknown)
	{
		isANewMap = false;
		updatedDimensions[WEST %4] = x;
		updatedDimensions[EAST %4] = x;
		updatedDimensions[NORTH % 4] = y;
		updatedDimensions[SOUTH %4] = y;
	}
	else
	{		
		if(x <  updatedDimensions[WEST %4] )
		{	
			if(newValue != unknown)
				updatedDimensions[WEST %4] = x;
		}
		else if(x >  updatedDimensions[EAST %4])
		{	
			if(newValue != unknown)
				updatedDimensions[EAST %4] = x;
		}
		if(y > updatedDimensions[NORTH % 4])
		{
			if(newValue != unknown)
				updatedDimensions[NORTH % 4] = y;
		}
		else if(y <  updatedDimensions[SOUTH %4] && newValue != unknown)
		{	
			updatedDimensions[SOUTH %4] = y;
		}
	}
    errorVal = 0;
	
    return true;
}


//---------------------------------------------------------------------
//---------------------------------------------------------------------

template <class T>
void Grid3DNoFile<T>::copy(Grid3DNoFile<T>* mapToCopy)
{
    T newVal = 0;
    long North, South, East, West, height;   
	
	isANewMap = true;
	
    North = mapToCopy->getUpdatedDimensions(NORTH);
    South = mapToCopy->getUpdatedDimensions(SOUTH);
    East = mapToCopy->getUpdatedDimensions(EAST);
    West = mapToCopy->getUpdatedDimensions(WEST);
	
    height = mapToCopy->getDimensions(ABOVE);

	T* row = 0;

	LOG<<"Going to copy map with dimensions west = "<<West<<",east="<<East<<",north="<<North<<",south="<<South;
	LOG<<"block size is "<<blockSize<<", unknown is "<<unknown;
    //if the maps are different heights, delete this map, change it's height, and rebuild it
    if(height != this->blockHeight - 1)
    {
		blockHeight = height;
		reset();
		row = new T[East - West +1];
		
		for(int y = North; y> South; y--)
		{
			for(int z = 0; z < blockHeight; z++)
			{
				mapToCopy->copyRow(row,y,West,East,z);
				for(int x = West; x< East; x++)
				{
					//newVal = mapToCopy->getGridRef(x,y,z);
					newVal = row[x - West];
					if(newVal != unknown) //no point updating the map with a value it has by default
						updateGridRef(newVal,x,y,z);
					
				}
			}
		}
		delete[] row;
		row = 0;
			
    }
    else //if the maps are the same height, no need to delete this one, just overwrite it
    {		
		row = new T[East - West +1];

		if(North > getDimensions(NORTH) && South > getDimensions(SOUTH)
			&& East > getDimensions(EAST) && West > getDimensions(WEST))
		{	
			LOG<<"New map is bigger, and same height, overwriting this map";
			//if the new map is bigger in all dimensions than the old one
			//there is no need to reset() the current map, just overwrite it
			for(int y = North; y> South; y--)
			{
				for(int z = 0; z < blockHeight; z++)
				{
					mapToCopy->copyRow(row,y,West, East,z);
					for(int x = West; x< East; x++)
					{						
					//	updateGridRef(mapToCopy->getGridRef(x,y,z),x,y,z);
						updateGridRef(row[x - West],x,y,z);						
					}
				}
			}
		}
		else
		{
			reset();
			for(int y = North; y>= South; y--)
			{
				for(int z = 0; z < blockHeight; z++)
				{
					mapToCopy->copyRow(row,y,West,East,z);
					for(int x = West; x<= East; x++)
					{
						//newVal = mapToCopy->getGridRef(x,y,z);
						newVal = row[x - West];
						
						if(newVal != unknown) //no point updating the map with a value it has by default
						{
							updateGridRef(newVal,x,y,z);											
						}
					}
				}
			}
			
		}
		delete[] row;
		row = 0;
    }
	
}

//note that the map passed in to this method is destroyed.  If you don't want the
//map to be destroyed, use the copy method
template <class T>
void Grid3DNoFile<T>:: clone(Grid3DNoFile<T>* mapToClone)
{
	if(myMap != 0)
	{
		reset();
		delete myMap;
	}

	*this = *mapToClone;

	mapToClone->init(blockSize,1,unknown,blockHeight);
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------

template <class T>
bool Grid3D<T>::save(char* filename)
{
    if(filename == 0)//if no file name was given
    {
		return false;
    }
	
    long north = this->getUpdatedDimensions(NORTH);
    long south = this->getUpdatedDimensions(SOUTH);
    long east = this->getUpdatedDimensions(EAST);
    long west = this->getUpdatedDimensions(WEST);
    long above = this->getDimensions(ABOVE);
	
    ofstream outstr;
    outstr.open(filename);
	
    if(!outstr)//file opening failed for some reason
    {
		return false;
    }
	
    //save the dimensions of the map
    outstr<<north<<' '<<south<<' '<<east<<' '<<west<<' '<<above<<' '<<blockSize<<endl;
	
    for(long y = north; y>= south; y--)
    {
		for(long x = west; x <= east; x++)
		{
			for(long z = 0; z<= above; z++)
				outstr<<getGridRef(x,y,z)<<' ';
		}
		outstr<<endl;
    }
	
	
    outstr.close();
    return true;
}


//---------------------------------------------------------------------
//---------------------------------------------------------------------

template <class T> 
bool Grid3D<T>::load(char* filename)
{
	long north=-1, south=-1, east=-1, west=-1, above = -1;

	bool fileFinished = false;
	
    T num;    
	long blockSz = 0;

    if(filename == 0)//if no file name was given
    {
		return false;
    }   
	
	char fileBuffer[256];

	char tempString[100];
	
    ifstream instr;
	instr.setbuf(fileBuffer,255);
    instr.open(filename);	
	
    if(!instr)//if the file didn't open
    {
		return false;
    }
	

	instr>>tempString;
	if(!SosUtil::is_numeric(tempString)) return false;
	north = atol(tempString);


	instr>>tempString;
	if(!SosUtil::is_numeric(tempString)) return false;
	south = atol(tempString);

	instr>>tempString;
	if(!SosUtil::is_numeric(tempString)) return false;
	east = atol(tempString);

	instr>>tempString;
	if(!SosUtil::is_numeric(tempString)) return false;
	west = atol(tempString);

	instr>>tempString;
	if(!SosUtil::is_numeric(tempString)) return false;
	above = atol(tempString);

	instr>>tempString;
	if(!SosUtil::is_numeric(tempString)) return false;
	blockSz = atol(tempString);//don't take block size from the file - ignore it
		
    if(above != blockHeight - 1)
    {
		blockHeight = above + 1;
		dimensions[ABOVE %6] = blockHeight;
		
    }

	updatedDimensions[NORTH % 4] = 0;
    updatedDimensions[SOUTH % 4] = 0;
    updatedDimensions[EAST % 4] = 0;
    updatedDimensions[WEST % 4] = 0;

	reset();
	
    for(long y = north; y>= south && !fileFinished && !instr.eof(); y--)
    {
		for(long x = west; x <= east && !fileFinished && !instr.eof(); x++)
		{
			for(long z = 0; z<= above && !fileFinished && !instr.eof(); z++)
			{
				instr>>num;
				updateGridRef(num, x,y,z);				
			}
		}
    }

	updatedDimensions[NORTH % 4] = north;
    updatedDimensions[SOUTH % 4] = south;
    updatedDimensions[EAST % 4] = east;
    updatedDimensions[WEST % 4] = west;
	
    instr.close();
    return true;
	
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------

template <class T>
long Grid3DNoFile<T>::getDimensions(int direction)
{
    errorVal = 0;
    switch(direction)
    {
	case NORTH: return dimensions[NORTH % 6]-1;
		break;
	case SOUTH: return dimensions[SOUTH % 6];
		break;
	case EAST:  return dimensions[EAST % 6] -1;
		break;
	case WEST:  return dimensions[WEST % 6];
		break;
	case ABOVE: return dimensions[ABOVE % 6] - 1;
		break;
	case BELOW: return dimensions[BELOW % 6];
		break;
	default:	errorVal = 0;
		return -1;
    }
}

template <class T>
void Grid3DNoFile<T>:: setDimensions(long west,long north,long east,long south)
{
	updatedDimensions[NORTH % 4] = north;
	updatedDimensions[SOUTH % 4] = south;
	updatedDimensions[EAST % 4]  = east;
	updatedDimensions[WEST % 4]  = west;
	isANewMap = false;
}


//---------------------------------------------------------------------
//---------------------------------------------------------------------

template <class T>
long Grid3DNoFile<T>::getUpdatedDimensions(int direction)
{
    errorVal = 0;
    switch(direction)
    {
	case NORTH: return updatedDimensions[NORTH % 4];
		break;
	case SOUTH: return updatedDimensions[SOUTH % 4];
		break;
	case EAST:  return updatedDimensions[EAST % 4];
		break;
	case WEST:  return updatedDimensions[WEST % 4];
		break;
	default:	errorVal = 0;
		return -1;
    }	
}

template <class T>
void Grid3DNoFile<T>:: getAllUpdatedDimensions(long& west,long& north,long&east,long&south)
{
	north = updatedDimensions[NORTH % 4];
	south = updatedDimensions[SOUTH % 4];
	east  = updatedDimensions[EAST % 4];
	west  = updatedDimensions[WEST % 4];
}


//---------------------------------------------------------------------
//---------------------------------------------------------------------

template <class T>
void Grid3DNoFile<T>::appendBlock(GridBlock<T>* original_block, GridBlock<T>* new_block, int direction)
{
    errorVal = 0;
    switch(direction)
    {
	case NORTH:
		original_block->north = new_block;
		new_block->south = original_block;
		new_block->globOrigin[XX] = original_block->globOrigin[XX];
		new_block->globOrigin[YY] = original_block->globOrigin[YY] + blockSize;
		break;
		
	case SOUTH:
		original_block->south = new_block;
		new_block->north = original_block;
		new_block->globOrigin[XX] = original_block->globOrigin[XX];
		new_block->globOrigin[YY] = original_block->globOrigin[YY] - blockSize;
		break;
		
	case WEST:
		original_block->west = new_block;
		new_block->east = original_block;
		new_block->globOrigin[XX] = original_block->globOrigin[XX] - blockSize;
		new_block->globOrigin[YY] = original_block->globOrigin[YY];
		
		break; 
	case EAST:
		original_block->east = new_block;
		new_block->west = original_block; 
		new_block->globOrigin[XX] = original_block->globOrigin[XX] + blockSize;
		new_block->globOrigin[YY] = original_block->globOrigin[YY];
		break; 		
    }
}


//---------------------------------------------------------------------
//---------------------------------------------------------------------

template <class T>
int Grid3DNoFile<T>::growMap(int times, int direction)
{
    errorVal = 0;
    GridBlock<T>* current;
    GridBlock<T>* new_block;
	
    long counter = 0;
	
    for(int i = 0; i<times; i++)
    {	
		current = myMap;//make current point to the original block at point (0,0)
		
		switch (direction)
		{
		case 0: growMap(times, NORTH);			
			growMap(times, SOUTH);			
			growMap(times, EAST);			
			growMap(times, WEST);			
			i = times;
			break;
			
			//grow the map by one block northwards
		case NORTH:					
			current = northWestBlock;

			new_block = newBlock();
			northWestBlock = northEastBlock = new_block;
			appendBlock(current, new_block, NORTH);
			dimensions[NORTH %4] = new_block->globOrigin[YY] + blockSize;

			current = current->east;
			
			while(current != 0)
			{
				new_block = newBlock();

				northEastBlock = new_block;
				appendBlock(current, new_block, NORTH);				
								
				appendBlock(current->west->north,new_block, EAST);				
				
				current = current->east;
			}
			break;
			
		case SOUTH:
			//find the southernmost block				
			current = southWestBlock;
			new_block = newBlock();
			southWestBlock = southEastBlock = new_block;
			appendBlock(current, new_block, SOUTH);
			dimensions[SOUTH %4] = new_block->globOrigin[YY];

			current = current->east;
			
			while(current != 0)
			{
				new_block = newBlock();
				southEastBlock = new_block;
				appendBlock(current, new_block, SOUTH);
		
				appendBlock(current->west->south,new_block, EAST);
			
				current = current->east;				
			}
			break;
			
		case EAST:			
			current = northEastBlock;
			new_block = newBlock();
			northEastBlock = southEastBlock = new_block;
			appendBlock(current, new_block, EAST);
			dimensions[EAST %4] = new_block->globOrigin[XX] + blockSize;
			current = current->south;	

			while(current != 0)
			{
				new_block = newBlock();
				southEastBlock = new_block;
				appendBlock(current, new_block, EAST);
				
				appendBlock(current->north->east,new_block, SOUTH);
			
				current = current->south;				
			}

			break;
			
		case WEST:
			current = northWestBlock;
			new_block = newBlock();
			appendBlock(current, new_block, WEST);
			northWestBlock = southWestBlock = new_block;
			dimensions[WEST %4] = new_block->globOrigin[XX];
			current = current->south;	
			
			while(current != 0)
			{		  
				new_block = newBlock();
				southWestBlock = new_block;
				appendBlock(current, new_block, WEST);	
				
				appendBlock(current->north->west,new_block, SOUTH);
								
				current = current->south;				
			}
			
			break;
		}//end of switch
    }//end of for loop
	
    return 0;
	
	
}



//---------------------------------------------------------------------
//---------------------------------------------------------------------

template <class T>
GridBlock<T> * Grid3DNoFile<T>::newBlock()
{	
    errorVal = 0;
    GridBlock<T> *ptr;
	
    ptr = new GridBlock<T>(blockSize, unknown, blockHeight, unknownArray);
	
    return ptr;
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------

template <class T>
void Grid3DNoFile<T>::reset()
{
    GridBlock<T>* current = myMap;

	isANewMap = true;

	lastAccessedBlock = 0;
	
    //go to the northermost block
    while(current->north != 0)
    {
		current = current->north;
    }
    //go to the north-westernmost block
    while(current->west != 0)
    {
		current = current->west;
    }
	
    //now delete the blocks from west to east, starting at the northernmost
    //row, and working your way south
    while(1)
    {
		while(1)
		{  
			if(current->east != 0) 
			{
				current = current->east;
				delete current->west;
			}
			else
				break;
		}
		//go south one level, then all the way to the west
		if(current->south !=0)
		{
			current = current->south;
			delete current->north;
			while(current->west != 0)
			{
				current = current->west;
				delete current->east;
			}
		}
		else
		{
			delete current;
			break;
		}
		
		if(current->south !=0)
		{
			current = current->south;
			delete current->north;
		}
		else
		{
			delete current;
			break;
		}
    }
	
    myMap = newBlock(); //create a new block for the map to grow from
    //all other functions assume that this exists, so this step is necessary
	
    myMap->globOrigin[XX] = 0;
    myMap->globOrigin[YY] = 0;
	
    dimensions[NORTH %4] = blockSize;
    dimensions[EAST %4]  = blockSize;
    dimensions[WEST %4]  = 0;
    dimensions[SOUTH %4] = 0;
	dimensions[ABOVE % 6] = blockHeight;
    dimensions[BELOW % 6] = 0;

	updatedDimensions[NORTH % 4] = 0;
    updatedDimensions[SOUTH % 4] = 0;
    updatedDimensions[EAST % 4] = 0;
	updatedDimensions[WEST % 4] = 0;

	northWestBlock = southWestBlock = northEastBlock = southEastBlock = myMap;	
}



//---------------------------------------------------------------------
//---------------------------------------------------------------------


template <class T>
GridBlock<T>* Grid3DNoFile<T>::findBlock(long x, long y)
{
    errorVal = 0;
    GridBlock<T> *current;// = myMap; //set the pointer to look at the original map block

	if(lastAccessedBlock == 0)
	{
		current = myMap;//set the pointer to look at the original map block
	}
	else
	{
		current = lastAccessedBlock;//set the pointer to look at the last GridBlock accessed

		//if(x >= current->globOrigin[XX] && x < current->globOrigin[XX] + blockSize
		//	&& y >= current->globOrigin[YY] && y < current->globOrigin[YY] + blockSize)
		//{
			
		//	lastAccessedBlock = current;
		//	return current;
		//}
	}
	
    while(1)
    {
		//if the grid reference we're looking for is in this current grid
		if(x >= current->globOrigin[XX] && x < current->globOrigin[XX] + blockSize
			&& y >= current->globOrigin[YY] && y < current->globOrigin[YY] + blockSize)
		{
			
			lastAccessedBlock = current;
			return current;
		}	
		
		//if the grid point we're looking for is to the north of the current grid block
		//change the pointer 'current' to point to the grid block to the north of the
		//current grid block, and so on for south, east and west
		if(x >= current->globOrigin[XX] + blockSize)//check west first
		{
			if(current->east == 0)
			{	errorVal = EAST;
			return 0;
			}
			else 
			{	current = current->east;}
		}
		else if(y >= current->globOrigin[YY] + blockSize)//check north second
		{
			if( current->north == 0)
			{	
				errorVal = NORTH;
				return 0;
			}
			else
			{	current = current->north;}
		}
		else if(y < current->globOrigin[YY]) //check south third
		{	
			if(current->south == 0)
			{
				errorVal = SOUTH;
				return 0;
			}
			else
			{ current = current->south;}
		}
		else if(y >= current->globOrigin[YY] + blockSize)//check north 
		{
			if( current->north == 0)
			{	
				errorVal = NORTH;
				return 0;
			}
			else
			{	current = current->north;}
		}		
		else if(x < current->globOrigin[XX])//check east last
		{
			if(current->west ==0)
			{	
				errorVal = WEST;
				return 0;
			}
			else
			{	current = current->west;}
		}
    }	
}

template <class T>
void Grid3DNoFile<T>::crop(long west,long north,long east,long south)
{
	if(myMap == 0)
		return;

	LOG<<"Grid3DNoFile cropping ("<<west<<","<<north<<")->("<<east<<","<<south<<")";

	lastAccessedBlock = 0;
	long temp = 0;
	if(west > east)
	{
		temp = west;
		west = east;
		east = temp;
	}
	if(north < south)
	{
		temp = north; 
		north = south;
		south = temp;
	}



	if(north > getUpdatedDimensions(NORTH))
		north = getUpdatedDimensions(NORTH);
	if(south < getUpdatedDimensions(SOUTH))
		south = getUpdatedDimensions(SOUTH);
	if(east > getUpdatedDimensions(EAST))
		east = getUpdatedDimensions(EAST);
	if(west < getUpdatedDimensions(WEST))
		west = getUpdatedDimensions(WEST);

	LOG<<"northEastBlock = "<<northEastBlock<<" ("<<northEastBlock->globOrigin[XX]<<","<<northEastBlock->globOrigin[YY]<<")";
	
	LOG<<"northWestBlock = "<<northWestBlock<<" ("<<northWestBlock->globOrigin[XX]<<","<<northWestBlock->globOrigin[YY]<<")";
	LOG<<"southEastBlock = "<<southEastBlock<<" ("<<southEastBlock->globOrigin[XX]<<","<<southEastBlock->globOrigin[YY]<<")";
	LOG<<"southWestBlock = "<<southWestBlock<<" ("<<southWestBlock->globOrigin[XX]<<","<<southWestBlock->globOrigin[YY]<<")";

	LOG<<"Current dimensions are: WEST = "<<getUpdatedDimensions(WEST);
	LOG<<"\tEAST="<<getUpdatedDimensions(EAST);
	LOG<<"\tNORTH="<<getUpdatedDimensions(NORTH);
	LOG<<"\tSOUTH="<<getUpdatedDimensions(SOUTH);
		

	//first delete all blocks on the west side that are no longer needed
	GridBlock<T>* current = 0;
	
	GridBlock<T>* prev = current;

	current = northWestBlock;

	LOG<<"Going to delete the west side"<<endl;
	
	while(current != 0 && current->globOrigin[XX] + blockSize -1 < west)
	{
		northWestBlock = northWestBlock->east;
		southWestBlock = southWestBlock->east;
		prev = current;
		while(current != 0)
		{
			current = current->south;
			if(prev->east != 0)
			{
				prev->east->west = 0;
			}
			LOG<<"Deleted "<<prev<<" ("<<prev->globOrigin[XX]<<","<<prev->globOrigin[YY]<<")";

			if(prev == northEastBlock)
				northEastBlock = northEastBlock->west;

			if(prev == southEastBlock)
				southEastBlock = southEastBlock->west;

			delete prev;
			prev = current;
		}
		current = northWestBlock;
		
	}
	
	LOG<<"Going to delete the east side"<<endl;
	LOG<<"northEastBlock = "<<northEastBlock<<endl;
	LOG<<"southEastBlock = "<<southEastBlock<<endl;	

	//now delete all grid blocks to the east of the crop area
	current = northEastBlock;
	while(current != 0 && current->globOrigin[XX] > east)
	{
		LOG<<"going to delete column "<<current->globOrigin[XX]<<endl;
		LOG<<"northEastBlock = "<<northEastBlock<<endl;
		LOG<<"southEastBlock = "<<southEastBlock<<endl;

		northEastBlock = northEastBlock->west;
		southEastBlock = southEastBlock->west;
		prev = current;
		while(current != 0)
		{
			LOG<<"current->globOrigin[X] = "<<current->globOrigin[XX]<<", current->globOrigin[Y] = "<<current->globOrigin[YY];
			current = current->south;
			if(prev->west != 0)
			{
				LOG<<"Set prev->west->east to 0";
				prev->west->east = 0;
			}

			
			if(prev == northWestBlock)
				northWestBlock = northWestBlock->east;

			if(prev == southWestBlock)
				southWestBlock = southWestBlock->east;

			delete prev;
			prev = current;
		}
		current = northEastBlock;
		
	}

	
	LOG<<"Going to delete the north side"<<endl;

	//now delete all grid blocks to the north of the crop area
	current = northWestBlock;
	while(current != 0 && current->globOrigin[YY] > north)
	{		
		northWestBlock = northWestBlock->south;
		northEastBlock = northEastBlock->south;
		prev = current;
		while(current != 0)
		{
			current = current->east;
			if(prev->south != 0)
			{
				prev->south->north = 0;
			}

			if(prev == southEastBlock)
				southEastBlock = 0;

			if(prev == southWestBlock)
				southWestBlock = 0;

			delete prev;
			prev = current;
		}
		current = northWestBlock;
	}

	
	LOG<<"Going to delete the south side"<<endl;

	//now delete all grid blocks to the south of the crop area
	current = southWestBlock;
	while(current != 0 && current->globOrigin[YY] > north)
	{		
		southWestBlock = southWestBlock->north;
		southEastBlock = southEastBlock->north;
		prev = current;
		while(current != 0)
		{
			current = current->east;
			if(prev->north != 0)
			{
				prev->north->south = 0;
			}

			if(prev == northEastBlock)
				northEastBlock = 0;

			if(prev == northWestBlock)
				northWestBlock = 0;


			delete prev;
			prev = current;
		}
		current = southWestBlock;
	}

	myMap = 0;
	if(northWestBlock != 0)
	{
		dimensions[NORTH %4] = northWestBlock->globOrigin[YY] + blockSize;
		dimensions[WEST %4] = northWestBlock->globOrigin[XX];
		myMap = northWestBlock;
	}
	else
	{
		dimensions[NORTH %4] = 0;//this is an error - should not happen
		dimensions[WEST  %4] = 0;
	}

	if(southEastBlock != 0)
	{
		dimensions[EAST  % 4] = southEastBlock->globOrigin[XX] + blockSize;
		dimensions[SOUTH % 4] = southEastBlock->globOrigin[YY] ;
		myMap = southEastBlock;
	}
	else
	{
		dimensions[EAST  % 4] = 0;//this is an error - should not happen
		dimensions[SOUTH % 4] = 0;
	}
	if(myMap == 0)
	{
		myMap = newBlock();
		reset();
	}

	for(long x = dimensions[WEST % 4]; x < dimensions[EAST % 4]; x++)
	{
		for(long y = dimensions[SOUTH % 4]; y < dimensions[NORTH%4]; y++)
		{
			if(x < west || x > east || y < south || y > north)
			{
				for(long z = 0; z < blockHeight; z++)
					updateGridRef(unknown,x,y,z);
			}
		}
	}

	setDimensions(west,north,east,south);

}


template <class T>
void Grid3DNoFile<T>::translate(long xDist, long yDist)
{
	GridBlock<T>* current = northWestBlock;	
	GridBlock<T>* westMost = current;
	
	while(westMost != 0)
	{
		while(current != 0)
		{
			current->globOrigin[XX] += xDist;
			current->globOrigin[YY] += yDist;
			current = current->east;
		}
		westMost = westMost->south;
		current = westMost;
	}
	
	updatedDimensions[NORTH % 4] += yDist;
	updatedDimensions[SOUTH % 4] += yDist;	
	updatedDimensions[WEST % 4]  += xDist;
	updatedDimensions[EAST % 4]  += xDist;

	dimensions[NORTH % 4] += yDist;
	dimensions[SOUTH % 4] += yDist;	
	dimensions[WEST  % 4] += xDist;
	dimensions[EAST  % 4] += xDist;
}

template <class T>
bool Grid3DNoFile<T>::copyRow(T* arrayRef, long y, long fromX, long toX, long z)
{
	if(arrayRef == 0 || z > blockHeight - 1 || fromX > toX)
    {
		return false;
    }	

	//if the row is completely outside the map, then just fill it up with the 
	//default value
	if(y < dimensions[SOUTH%6] || y > dimensions[NORTH%6]-1
		|| (toX < dimensions[WEST%6]) || (fromX >= dimensions[EAST%6]))
	{
		long arrayLength = toX - fromX +1;
		for(int i = 0; i< arrayLength; i++)
		{
			arrayRef[i] = unknown;
		}

		return true;
	}
	long west = dimensions[WEST%6];

	//if the fromX is to the left of the map, fill in the values up to the first cell
	while(fromX < west)
	{
		fromX++;
		arrayRef[0] = unknown;
		arrayRef++;
	}

	GridBlock<T> *current = 0;
	current = findBlock(fromX,y);

	if(current == 0)
		return false;

	bool retval = true;
	int numCopied = 0;
	long tempFromX = 0, tempToX = 0;

	tempFromX = int(fabs(current->globOrigin[XX] - fromX));
	long yVal = long(fabs(current->globOrigin[YY] - y));

	while(current != 0 && current->globOrigin[XX] <= toX)
	{		
		tempToX = SosUtil::minVal(blockSize -1L,toX - current->globOrigin[XX]);
		retval = current->copyRow(arrayRef + numCopied,yVal,
			tempFromX,tempToX,z);

		if(!retval)
		{			
			return false;
		}
		numCopied += (tempToX - tempFromX) + 1; 

		current = current->east;
		if(current != 0)
		{
			tempFromX = 0;
		}		
	}
	arrayRef+= numCopied;

	long east = dimensions[EAST%6];
	while(toX > east)
	{
		toX--;
		arrayRef[0] = unknown;
		arrayRef++;
	}

	return true;
}


#endif  


