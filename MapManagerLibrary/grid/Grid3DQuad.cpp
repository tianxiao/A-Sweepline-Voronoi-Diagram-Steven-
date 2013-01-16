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


#include "Grid3DQuad.h"
#include <iostream.h>
#include "stdlib.h"
#include <string.h>

template Grid3DQuad<double>;
template Grid3DQuad<int>;
template Grid3DQuad<float>;
template Grid3DQuad<long>;



template <class T>
Grid3DQuad<T>::Grid3DQuad()
{
    unknown = 0; 
    blockSize = DEFAULT_BLOCKSIZE_QUAD;
    blockHeight = DEFAULT_BLOCKHEIGHT_QUAD;

	assert(blockHeight > 0);

    myMap = newBlock();
    reset();
    
    //make the map have a radius of 'radius + 1' number of blocks
    growMap(1);
	
    dimensions[ABOVE % 6] = blockHeight;
    dimensions[BELOW % 6] = 0;
	
    updatedDimensions[NORTH % 4] = 0;
    updatedDimensions[SOUTH % 4] = 0;
    updatedDimensions[EAST % 4] = 0;
    updatedDimensions[WEST % 4] = 0;
    errorVal = 0;
	isANewMap = true;//this is used with the updatedDimensions. if it is true, then the first
	//cell to be updated becomes the newly updated DImensions for NORTH, SOUTH, EAST and WEST


	//when a user accesses a grid cell, it is very likely that it will be close to the cell they 
	//last accessed, so to speed up the search, instead of searching from the origin (0,0) every time
	//we search from the last position accessed
	lastAccessedBlock = 0;
	
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------

template <class T>
Grid3DQuad<T>::Grid3DQuad(int blocksize, int radius, T Unknown, int blockheight)
{
    unknown = Unknown;
    blockSize = blocksize;
    blockHeight = blockheight;

	assert(blockHeight > 0);
	
    myMap = newBlock();
    //reset();
    
    //make the map have a radius of 'radius + 1' number of blocks
    growMap(radius);
	
    dimensions[ABOVE % 6] = blockHeight;
    dimensions[BELOW % 6] = 0;

	updatedDimensions[NORTH % 4] = 0;
    updatedDimensions[SOUTH % 4] = 0;
    updatedDimensions[EAST % 4] = 0;
    updatedDimensions[WEST % 4] = 0;
	
    errorVal = 0;

	lastAccessedBlock = 0;
	
	
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------

template <class T>
Grid3DQuad<T>::~Grid3DQuad()
{
    if(myMap != 0)
    {
		reset();
		delete myMap;
    }
	
    myMap = 0;
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------

template <class T>
T Grid3DQuad<T>::getGridRef(long x, long y, long z)
{
    errorVal = 0;
    T retval;
    GridBlockQuad<T> *current;
	
    if(z > blockHeight - 1)
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
bool Grid3DQuad<T>::updateGridRef( T newValue, long x, long y, long z )
{
    int noOfBlocks = 0;	
    
    GridBlockQuad<T>* current = 0;
	
    if(z > blockHeight - 1)
    {
		return false;
    }
	
    current = findBlock(x,y); 
	
    //    cout<<"\nfindBlock("<<x<<","<<y<<") returned "<<current<<endl;
	
    while(current == 0)
    {		
		current = myMap;
		switch(errorVal) //decide which direction to grow the map in
		{	
		case NORTH: //find the northernmost block of the map and calculate how many 
			//blocks are needed to reach the new one
			
			while(current->north != 0)
			{
				current = current->north;
			}
			noOfBlocks = int(fabs(y - current->globOrigin[YY])/ blockSize);
			
			if(noOfBlocks == 0) //if noOfBlocks is rounded down because of its conversion to integer
				noOfBlocks++;   //we need to increment it
			
			//		cout<<"\nAbout to grow the map NORTH"<<endl;
			growMap(noOfBlocks, NORTH);
			break;
			
		case SOUTH: //find the southernmost block of the map and calculate how many 
			//blocks are needed to reach the new one
			
			while(current->south != 0)
			{
				current = current->south;
			}
			noOfBlocks = int(fabs(y - current->globOrigin[YY])/ blockSize);
			
			if(noOfBlocks == 0) 
				noOfBlocks++;
			
			//		cout<<"\nAbout to grow the map SOUTH"<<endl;
			growMap(noOfBlocks, SOUTH);
			break;
			
		case EAST: //find the easternmost block of the map and calculate how many 
			//blocks are needed to reach the new one
			
			while(current->east != 0)
			{
				current = current->east;
			}
			
			noOfBlocks = int(fabs(x - current->globOrigin[XX])/ blockSize);
			
			if(noOfBlocks == 0) 
				noOfBlocks++;
			
			//		cout<<"\nAbout to grow the map EAST"<<endl;
			growMap(noOfBlocks, EAST);
			break;
			
		case WEST: //find the westernmost block of the map and calculate how many 
			//blocks are needed to reach the new one
			
			while(current->west != 0)
			{
				current = current->west;
			}
			
			noOfBlocks = int(fabs(x - current->globOrigin[XX])/ blockSize);
			
			if(noOfBlocks == 0) 
				noOfBlocks++;
			
			//		cout<<"\nAbout to grow the map WEST"<<endl;
			growMap(noOfBlocks, WEST);
			break;
			
		default: 
			//   cout<<"\nAbout to grow the map IN EVERY DIRECTION"<<endl;
			growMap(1); //if the direction has not been reported, just grown the whole map
			//in every direction by one block, and try again
			break;
		}
		
		current = findBlock(x,y);
		//	cout<<"\nfindBlock("<<x<<","<<y<<") returned "<<current<<endl;
		
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
		if(x <  updatedDimensions[WEST %4] && newValue != unknown)
		{	
			updatedDimensions[WEST %4] = x;
		}
		if(x >  updatedDimensions[EAST %4] && newValue != unknown)
		{	
			updatedDimensions[EAST %4] = x;
		}
		if(y > updatedDimensions[NORTH % 4] && newValue != unknown)
		{
			updatedDimensions[NORTH % 4] = y;
		}
		if(y <  updatedDimensions[SOUTH %4] && newValue != unknown)
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
void Grid3DQuad<T>::copy(Grid3DQuad<T>* mapToCopy)
{
    T newVal;
	newVal= 0;
    long North, South, East, West, height;   
	
	isANewMap = true;


	
    North = mapToCopy->getDimensions(NORTH);
    South = mapToCopy->getDimensions(SOUTH);
    East = mapToCopy->getDimensions(EAST);
    West = mapToCopy->getDimensions(WEST);
	
    height = mapToCopy->getDimensions(ABOVE);
	
    //if the maps are different heights, delete this map, change it's height, and rebuild it
    if(height != this->blockHeight - 1)
    {
		blockHeight = height;
		reset();
		
		for(int y = North; y> South; y--)
		{
			for(int x = West; x< East; x++)
			{
				for(int z = 0; z < blockHeight; z++)
				{
					newVal = mapToCopy->getGridRef(x,y,z);
					if(newVal != unknown) //no point updating the map with a value it has by default
						updateGridRef(newVal,x,y,z);
				}
			}
		}
    }
    else //if the maps are the same height, no need to delete this one, just overwrite it
    {		
		if(North > getDimensions(NORTH) && South > getDimensions(SOUTH)
			&& East > getDimensions(EAST) && West > getDimensions(WEST))
		{	//if the new map is bigger in all dimensions than the old one
			//there is no need to reset() the current map, just overwrite it
			for(int y = North; y> South; y--)
			{
				for(int x = West; x< East; x++)
				{
					for(int z = 0; z < blockHeight; z++)
					{
						updateGridRef(mapToCopy->getGridRef(x,y,z),x,y,z);
					}
				}
			}
		}
		else
		{
			reset();
			for(int y = North; y> South; y--)
			{
				for(int x = West; x< East; x++)
				{
					for(int z = 0; z < blockHeight; z++)
					{
						newVal = mapToCopy->getGridRef(x,y,z);
						if(newVal != unknown) //no point updating the map with a value it has by default
							updateGridRef(newVal,x,y,z);
					}
				}
			}
			
		}
    }
	
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------

template <class T>
bool Grid3DQuad<T>::save(char* filename)
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
		return 0;
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
bool Grid3DQuad<T>::load(char* filename)
{
	long north=-1, south=-1, east=-1, west=-1, above = -1;

	bool fileFinished = false;
	
    T num;    

    if(filename == 0)//if no file name was given
    {
		return false;
    }   

	//ofstream report;
	//report.open("f:\\temp\\gmapReport.txt");

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
	north = atol(tempString);

	instr>>tempString;
	south = atol(tempString);

	instr>>tempString;
	east = atol(tempString);

	instr>>tempString;
	west = atol(tempString);

	instr>>tempString;
	above = atol(tempString);

	instr>>tempString;
	blockSize = atol(tempString);
	
	//report<<"\nNorth = "<<north<<"\nAnd the fileBuffer = "<<fileBuffer<<endl;			
	//report<<"\nSouth = "<<south<<"\nAnd the fileBuffer = "<<fileBuffer<<endl;			
	//report<<"\nEast = "<<east<<"\nAnd the fileBuffer = "<<fileBuffer<<endl;		
	//report<<"\nWest = "<<west<<"\nAnd the fileBuffer = "<<fileBuffer<<endl;		
	//report<<"\nAbove = "<<above<<"\nAnd the fileBuffer = "<<fileBuffer<<endl;		
	//report<<"\nBlockSize = "<<blockSize<<"\nAnd the fileBuffer = "<<fileBuffer<<endl;

	
	

	//report<<"\nGot north = "<<north<<", south = "<<south<<", east = "<<east
	//	<<", west = "<<west<<endl;
	
	
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
	
    instr.close();
	
	//report.close();
    return true;
	
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------

template <class T>
long Grid3DQuad<T>::getDimensions(int direction)
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


//---------------------------------------------------------------------
//---------------------------------------------------------------------

template <class T>
long Grid3DQuad<T>::getUpdatedDimensions(int direction)
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

//---------------------------------------------------------------------
//---------------------------------------------------------------------

template <class T>
void Grid3DQuad<T>::appendBlock(GridBlockQuad<T>* original_block, GridBlockQuad<T>* new_block, int direction)
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
		
		//	    cout<<"\nOrig Block["<<original_block->globOrigin[XX]<<","<<original_block->globOrigin[YY]
		//	<<" and newBlock["<<new_block->globOrigin[XX]<<","<<new_block->globOrigin[YY]
		//	<<"] and blocksize = "<<blockSize;
		
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
int Grid3DQuad<T>::growMap(int times, int direction)
{
    errorVal = 0;
    GridBlockQuad<T>* current;
    GridBlockQuad<T>* new_block;
	
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
			
			//find the topmost block
			while(current->north != 0)
			{
				current = current->north;
			}
			//find the north-western most block
			while(current->west != 0)
			{
				current = current->west;
			}
			
			do
			{
				new_block = newBlock();
				appendBlock(current, new_block, NORTH);
				dimensions[NORTH %4] = new_block->globOrigin[YY] + blockSize;
				//reportfile<<"dimensions[NORTH] = new_block->globOrigin[YY] + blocksize = "<<new_block->globOrigin[YY]<<" + "<<blocksize
				//<<" = "<<dimensions[NORTH %4]<<endl;
				
				if(current->west !=0)
				{
					if(current->west->north !=0)
					{
						appendBlock(current->west->north,new_block, EAST);
					}
					
				}
				current = current->east;
			}while(current != 0);
			break;
			
		case SOUTH:
			//find the southernmost block
			while(current->south != 0)
			{
				current = current->south;
			}
			//find the south-western most block
			while(current->west != 0)
			{
				current = current->west;
			}
			
			do
			{
				new_block = newBlock();
				appendBlock(current, new_block, SOUTH);
				dimensions[SOUTH %4] = new_block->globOrigin[YY];
				//reportfile<<"dimensions[SOUTH] = new_block->globOrigin[YY] = "<<new_block->globOrigin[YY]<<" = "<<dimensions[SOUTH %4]<<endl;
				
				if(current->west !=0)
				{
					if(current->west->south !=0)
					{
						appendBlock(current->west->south,new_block, EAST);
					}
					
				}
				
				current = current->east;				
			}while(current != 0);
			
			break;
			
		case EAST:
			//find the easternmost block
			while(current->east != 0)
			{
				current = current->east;
			}
			//find the north-eastern most block
			while(current->north != 0)
			{
				current = current->north;
			}
			
			do
			{
				new_block = newBlock();
				appendBlock(current, new_block, EAST);
				dimensions[EAST %4] = new_block->globOrigin[XX] + blockSize;
				
				
				if(current->north !=0)
				{
					if(current->north->east !=0)
					{
						appendBlock(current->north->east,new_block, SOUTH);
					}
				}
				
				current = current->south;				
			}while(current != 0);
			
			break;
			
		case WEST:
			//find the westernmost block
			while(current->west != 0)
			{
				counter++;
				current = current->west;
			}
			//find the north-western most block
			while(current->north != 0)
			{
				current = current->north;
			}
			
			do
			{		  
				new_block = newBlock();
				appendBlock(current, new_block, WEST);		    
				
				dimensions[WEST %4] = new_block->globOrigin[XX];
				//reportfile<<"dimensions[WEST] = new_block->globOrigin[XX]"<<new_block->globOrigin[XX]<<" = "<<dimensions[WEST %4]<<endl;
				
				if(current->north !=0)
				{
					if(current->north->west !=0)
					{
						appendBlock(current->north->west,new_block, SOUTH);
					}
				}
				
				current = current->south;				
			}while(current != 0);
			
			break;
	}//end of switch
    }//end of for loop
	
    return 0;
	
	
}



//---------------------------------------------------------------------
//---------------------------------------------------------------------

template <class T>
GridBlockQuad<T> * Grid3DQuad<T>::newBlock()
{
	
    errorVal = 0;
    GridBlockQuad<T> *ptr;
	
    ptr = new GridBlockQuad<T>(blockSize, unknown, blockHeight);
	
    return ptr;
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------

template <class T>
void Grid3DQuad<T>::reset()
{
    GridBlockQuad<T>* current = myMap;

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
	
}



//---------------------------------------------------------------------
//---------------------------------------------------------------------


template <class T>
GridBlockQuad<T>* Grid3DQuad<T>::findBlock(long x, long y)
{
    errorVal = 0;
    GridBlockQuad<T> *current;// = myMap; //set the pointer to look at the original map block

	if(lastAccessedBlock == 0)
	{
		current = myMap;//set the pointer to look at the original map block
	}
	else
	{
		current = lastAccessedBlock;//set the pointer to look at the last GridBlockQuad accessed
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
		if(y >= current->globOrigin[YY] + blockSize)
		{
			if( current->north == 0)
			{	
				errorVal = NORTH;
				return 0;
			}
			else
			{	current = current->north;}
		}
		else if(y < current->globOrigin[YY])
		{	
			if(current->south == 0)
			{
				errorVal = SOUTH;
				return 0;
			}
			else
			{ current = current->south;}
		}
		else if(x >= current->globOrigin[XX] + blockSize)
		{
			if(current->east == 0)
			{	errorVal = EAST;
			return 0;
			}
			else 
			{	current = current->east;}
		}
		else if(x < current->globOrigin[XX])
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
//#endif
