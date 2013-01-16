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
GridBlockQuad.cpp
defines the functions of the GridBlockQuad class
*/

#include "GridBlockQuad.h"

template GridBlockQuad<double>;
template GridBlockQuad<int>;
template GridBlockQuad<float>;
template GridBlockQuad<long>;

template <class T>
GridBlockQuad<T>::GridBlockQuad():blockSize(64), blockHeight(1), defaultVal(0)
{
	assert(blockSize % 4 == 0 || blockSize == 2 || blockSize == 1);		
	
	north=south=east=west=above=below=0; 
	globOrigin[XX]=globOrigin[YY]=0;
	
	myValues = new T[blockHeight];
	
	for(int i = 0; i<blockHeight; i++)
    {
		myValues[i] = defaultVal;
    }	
	
	topLeft = 0;
	topRight = 0;
	bottomLeft = 0;
	bottomRight = 0;
}


template <class T>
GridBlockQuad<T>::GridBlockQuad(long blocksize,T defaultval,long blockheight):blockSize(blocksize), blockHeight(blockheight)
{
	assert(blockSize % 4 == 0 || blockSize == 2 || blockSize == 1);
	int i = 0;
	
	north=south=east=west=above=below=0; 
	globOrigin[XX]=globOrigin[YY]=0;	
	
	defaultVal = defaultval;    
	
	myValues = new T[blockHeight];
	
	for(i = 0; i<blockHeight; i++)
    {
		myValues[i] = defaultVal;
    } 
	
	topLeft = 0;
	topRight = 0;
	bottomLeft = 0;
	bottomRight = 0;
}

template <class T>
GridBlockQuad<T>::~GridBlockQuad()
{
	delete []myValues;
	//  cout<<"\nDeleting block at ["<<globOrigin[XX]<<","<<globOrigin[YY]<<"]"<<endl;
			
	if(topLeft != 0)
		delete topLeft;
	
	if(topRight != 0)
		delete topRight;
	
	if(bottomLeft !=0)
		delete bottomLeft;
	
	if(bottomRight != 0)
		delete bottomRight;  
}

template <class T>
bool GridBlockQuad<T>::putVal(T value, long x, long y, long z)
{
	//  cout<<"\nin PutVal("<<x<<","<<y<<","<<z<<") and blockSize = "<<blockSize
	//      <<" and blockHeight = "<<blockHeight<<endl;
	if((x < 0 || y<0 || z<0) || (x>blockSize-1 || y>blockSize-1 || z > blockHeight-1))
    {
		//    cout<<"\nReturning false because [x,y] = ["<<x<<","<<y<<"] and ";
		return false;
    }
	
	//if the value should go in this gridBlock, then put it here
	if((x == globOrigin[XX] && y == globOrigin[YY]) || blockSize == 1)
    {
		//   cout<<"\nsetting my own (x,y,z) value to "<<value<<endl;
		myValues[z] = value;
		//  cout<<"\nMy values are now "<<myValues[z]<<endl;
		return true;
    }
	else
    {
		//   cout<<"\nGoing to a child GridBlock because my ["<<globOrigin[XX]<<","<<globOrigin[YY]<<"] != ["
		//	  <<x<<","<<y<<"]"<<endl;
		
		//now decide which child it goes to
		if(y >=  blockSize/2)
		{
			if(x < blockSize/2)//put it in the top left child
			{
				//   cout<<"\nGoing to the top left child"<<endl;
				if(topLeft == 0)
				{
					//	  cout<<"\nCreating the top left child"<<endl;
					topLeft = new GridBlockQuad(blockSize/2, defaultVal, blockHeight);
					topLeft->globOrigin[XX] = this->globOrigin[XX];
					topLeft->globOrigin[YY] = this->globOrigin[YY] + blockSize/2;
				}
				
				return topLeft->putVal(value, x, y - (topLeft->globOrigin[YY] - this->globOrigin[YY]), z);
			}
			else //put it in the top right child
			{
				//  cout<<"\nGoing to the top right child"<<endl;
				if(topRight == 0)
				{
					//	  cout<<"\nCreating the top right child"<<endl;
					topRight = new GridBlockQuad(blockSize/2, defaultVal, blockHeight);
					topRight->globOrigin[XX] = this->globOrigin[XX]  + blockSize/2;
					topRight->globOrigin[YY] = this->globOrigin[YY] + blockSize/2;
				}
				return topRight->putVal(value, x - (topRight->globOrigin[XX] - this->globOrigin[XX]), 
					y - (topRight->globOrigin[YY] - this->globOrigin[YY]), z);
			}
		}
		else
		{
			if(x < blockSize/2)//put it in the top left child
			{
				//cout<<"\nGoing to the bottom left child"<<endl;
				if(bottomLeft == 0)
				{
					// cout<<"\nCreating the bottom left child"<<endl;
					bottomLeft = new GridBlockQuad(blockSize/2, defaultVal, blockHeight);
					bottomLeft->globOrigin[XX] = this->globOrigin[XX];
					bottomLeft->globOrigin[YY] = this->globOrigin[YY];
				}
				
				return bottomLeft->putVal(value, x,y, z);
			}
			else //put it in the top right child
			{
				//cout<<"\nGoing to the bottom right child"<<endl;
				if(bottomRight == 0)
				{		  
					//cout<<"\nCreating the bottom right child"<<endl;
					bottomRight = new GridBlockQuad(blockSize/2, defaultVal, blockHeight);
					bottomRight->globOrigin[XX] = this->globOrigin[XX] + blockSize/2;
					bottomRight->globOrigin[YY] = this->globOrigin[YY];
				}
				return bottomRight->putVal(value, x - (bottomRight->globOrigin[XX] - this->globOrigin[XX]),y, z);
			}		
		}	    
    }	
	
	//	values[x][y][z] = value;
	//return true;
}

template <class T>
T GridBlockQuad<T>::getVal(long x, long y, long z)
{
	// cout<<"\nIn getVal("<<x<<","<<y<<","<<z<<")"<<endl;
	if((x < 0 || y<0 || z<0) || (x>blockSize-1 || y>blockSize-1 || z > blockHeight-1))
    {
		//cout<<"\ngetVal() returning defaultVal"<<endl;
		return 0;
    }
	else
    {
		if((x == globOrigin[XX] && y == globOrigin[YY]) || blockSize == 1)
		{
			//cout<<"\ngetVal() returning the value from this GridBlock"<<endl;
			//cout<<"\nReturning the values from memory position "<<myValues + z<<" : "<<*(myValues + z)<<endl;
			return *(myValues + z);
		}
		
		//now decide which child it goes to
		if(y >=  blockSize/2)
		{
			if(x < blockSize/2)//get it from  the top left child
			{
				//cout<<"\ngetVal() going to top left child"<<endl;
				if(topLeft == 0)
				{
					return myValues[z];
				}
				
				return topLeft->getVal(x , y - (topLeft->globOrigin[YY] - this->globOrigin[YY]), z);
			}
			else //get it from the top right child
			{
				//cout<<"\ngetVal() going to top right child"<<endl;
				if(topRight == 0)
				{
					return myValues[z];
				}
				return topRight->getVal(x - (topRight->globOrigin[XX] - this->globOrigin[XX])
					, y - (topRight->globOrigin[YY]-this->globOrigin[YY]), z);
			}
		}
		else
		{
			if(x < blockSize/2)//get it from the bottom left child
			{
				//cout<<"\ngetVal() going to bottom left child"<<endl;
				if(bottomLeft == 0)
				{
					return myValues[z];
				}
				
				return bottomLeft->getVal(x, y, z);
			}
			else //get it from the bottom right child
			{
				//cout<<"\ngetVal() going to bottom right child"<<endl;
				if(bottomRight == 0)
				{
					return myValues[z];
				}
				return bottomRight->getVal(x - (bottomRight->globOrigin[XX] - this->globOrigin[XX]),y, z);
			}		
		}
    }
	
}
