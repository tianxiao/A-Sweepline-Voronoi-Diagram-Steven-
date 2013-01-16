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
GridMap.cpp
defines the GridMap class, a child of the Grid class
*/

#include "SosUtil.h"
#include <fstream.h>
#include "GridMap.h" 


template GridMap<double>;
template GridMap<int>;
template GridMap<float>;
template GridMap<long>;


//default constructor
template <class T>
GridMap<T>::GridMap():Grid3D<T>()
{
	GET_FILE_LOG
	//LOGGING_OFF
	topLeftSet = false;
}

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------

//class constructor
template <class T>
GridMap<T>::GridMap(int blocksize, int radius, T Unknown):Grid3D<T>(blocksize, radius, Unknown, 1)
{	 
	//reportfile.open("c:\\temp\\gmapReport.txt");
	GET_FILE_LOG
	//LOGGING_OFF
	topLeftSet = false;
	
}

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------


//this constructor is used to directly copy one set of GridBlocks, without duplicating
//the data.  This will currently be used when resizing the map
template <class T>
GridMap<T>::GridMap(GridBlock<T>* newGridBlock, long *dim, long blocksize,T Unknown):Grid3D<T>(blocksize,1,Unknown,1)
{
	GET_FILE_LOG
	reset();//get rid of the map that the base class constructer initialised
	delete myMap;
	myMap = newGridBlock;
	
	dimensions[NORTH % 4] = dim[NORTH % 4];
	dimensions[SOUTH % 4] = dim[SOUTH % 4] ;
	dimensions[EAST % 4]  = dim[EAST % 4];
	dimensions[WEST % 4]  = dim[WEST % 4] ; 
	errorVal = 0;  
	topLeftSet = false;
}

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------

//class destructor - delete all the blocks we created
template <class T>
GridMap<T>::~GridMap()
{
	LOG<<"At start of ~GridMap()";
	if(myMap != 0) //if the map is not empty
	{
		reset();
		delete myMap;
	}
	myMap = 0;
	LOG<<"At end of ~GridMap()";
}

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------

//creates a copy of the map, reducing it's resolution by the factor of reduceFactor
//the only valid reduce values are 1, 4 and 9
//it can select the largest value of the 4/9 cells, or it can select the smallest value,
//decided by the value of valueToSelect

template <class T>
void GridMap<T>::copy(Grid3D<T>* mapToCopy,int reduceFactor,int valueToSelect)
{
	T maxVal=-10000;
	T minVal= 10000;
	T newVal = 0;
	long North=0, South=0, East=0, West=0, Above=0; 

	static bool northeast = true;

	if(mapToCopy == this)
	{
	reduceDimension(reduceFactor);
	}
	else
	{
	North = mapToCopy->getDimensions(NORTH);
	South = mapToCopy->getDimensions(SOUTH);
	East = mapToCopy->getDimensions(EAST);
	West = mapToCopy->getDimensions(WEST);
	Above= mapToCopy->getDimensions(ABOVE);

	unknown = mapToCopy->getUnknown();
		
	if(reduceFactor == 1)//don't reduce the dimesionality of the map, just copy
	{
		//use the copy function from the base class
		Grid3D<T>::copy(mapToCopy);
	}
						
	//a problem with reducing the dimesionality by 4 times is that there is no central block
	//so when we replace 4 blocks with one block, we shift the map in a certain direction,
	//for example north-east if we replace (0,0), (0,1), (1,0) and (1,1) with a single block.
	//the solution for this is to perform this action the first time as above, shifting the
	//map north-east, but if it is done a second time, shift the map south-west, and vice versa
	if(reduceFactor == 4)//reduce the dimensionality by 4 times, i.e 1 square where there
	{					 //used to be 4
		reset();

		if(northeast)
		{
		northeast = false;
				
		for(int y = North - 1; y> South -1; y-=2)
		{
			for(int x =West ; x< East; x+=2)
			{					
			maxVal = -10000;
			minVal =  10000;
			//find the largest/smallest value of the nine squares, and put that into the
			//new map
			for(int y2 = y+1; y2 > y-1; y2--)  //start looking from the top left of the 3x3 box
			{
				for(int x2 = x; x2<x+2; x2++)
				{
				newVal = mapToCopy->getGridRef(x2,y2);
				if(newVal > maxVal) //find largest value
					maxVal = newVal;
				if(newVal < minVal) //find smallest value
					minVal = newVal;
				}
							
			}
			if(maxVal != 0 && valueToSelect == LARGEST_VALUE) //no point updating the map with a value that it has by default
			{			   //this will make the map smaller if the original has many empty cells
				//near the border
				updateGridRef(maxVal,x/2,y/2);
							
			}
			if(minVal != 0 && valueToSelect == SMALLEST_VALUE) //no point updating the map with a value that it has by default
			{			   //this will make the map smaller if the original has many empty cells
				//near the border
				updateGridRef(minVal,x/2,y/2);
							
			}
						
			}
		}	
		}
		else
		{
		northeast = true;
				
		for(int y = North; y> South -1; y-=2)
		{
			for(int x =West +1; x< East+1; x+=2)
			{
			maxVal = -10000;
			minVal =  10000;
			//find the largest value of the nine squares, and put that into the
			//new map
			for(int y2 = y; y2 > y-2; y2--)  //start looking from the top left of the 3x3 box
			{
				for(int x2 = x-1; x2<x+1; x2++)
				{
				newVal = mapToCopy->getGridRef(x2,y2);
								
				if(newVal > maxVal) //find largest value
					maxVal = newVal;
				if(newVal < minVal) //find smallest value
					minVal = newVal;
				}
							
			}
			if(maxVal != 0 && valueToSelect == LARGEST_VALUE) //no point updating the map with a value that it has by default
			{			   //this will make the map smaller if the original has many empty cells
				//near the border
				updateGridRef(maxVal,x/2,y/2);				
			}
			if(minVal != 0 && valueToSelect == SMALLEST_VALUE) //no point updating the map with a value that it has by default
			{			   //this will make the map smaller if the original has many empty cells
				//near the border
				updateGridRef(minVal,x/2,y/2);				
			}
			}
		}	
		}
	}
		
		
	if(reduceFactor == 9)//reduce the dimensionality by 9 times, i.e 1 square where there used to be 9
	{		   
		reset();
		for(int y = North - 1; y> South; y-=3)
		{
		for(int x =West + 1; x< East; x+=3)
		{
			maxVal = -10000;
			minVal =  10000;
			//find the largest value of the nine squares, and put that into the
			//new map
			for(int y2 = y+1; y2 > y-2; y2--)  //start looking from the top left of the 3x3 box
			{
			for(int x2 = x-1; x2<x+2; x2++)
			{
				newVal = mapToCopy->getGridRef(x2,y2);
				//	outstr<<"\n("<<x2<<','<<y2<<") "<<newVal;
				if(newVal > maxVal) //find largest value
				maxVal = newVal;
				if(newVal < minVal) //find smallest value
				minVal = newVal;
			}
						
			}
			if(maxVal != 0 && valueToSelect == LARGEST_VALUE) //no point updating the map with a value that it has by default
			{			   //this will make the map smaller if the original has many empty cells
			//near the border
			updateGridRef(maxVal,x/3,y/3);
			}
			if(minVal != 0 && valueToSelect == SMALLEST_VALUE) //no point updating the map with a value that it has by default
			{			   //this will make the map smaller if the original has many empty cells
			//near the border
			updateGridRef(minVal,x/3,y/3);
			}
		}
		}
	} 
	}
}

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------


//reduceDimension() reduces the size of the map by a factor of reduceFactor
template <class T>
void GridMap<T>::reduceDimension(int reduceFactor, int valueToSelect)
{
	//create a new map, copy our map into it, then copy it back while shrinking it
	GridMap<T> *tempMap = new GridMap<T>(myMap, dimensions, blockSize, unknown);
	
	myMap = newBlock();
	
	copy(tempMap, reduceFactor, valueToSelect);
	
	delete tempMap;
}

template <class T>
bool GridMap<T>::copyRow(T* arrayRef, long y, long fromX, long toX)
{
	return Grid3D<T>::copyRow(arrayRef,y,fromX,toX,0);
}


//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------

/*
blurs or smooths the map 
the map uses a mask, or kernel, of constant value 'c' to smooth the map.
The algorithm works by first applying a vertical blur, then a horizontal one.
Instead of contantly recomputing the kernel, each time it is moved, the cell left behind
is subtracted from the total, and the cell newly covered are added to the total
*/
template <class T>
void GridMap<T>::boxBlur(int kernelSize, double boxVal)
{
	T tempProb=0;
	int x,y;	
	int k;
	
	GridMap<T>* oldMap;
	
	//copy the map to be blurred into another gridmap, without duplicating the data
	oldMap = new GridMap<T>(myMap,dimensions,blockSize, unknown);
	
	//remove any reference in this map to the old map to be blurred
	myMap = newBlock();
	this->reset();
	
	long Xmax, Ymax, Xmin, Ymin; //store the dimensions of the main map 
	
	if((kernelSize %2) == 0) kernelSize++; //kernelSize should be an odd number
	
	//k is half the height of the kernel, eg if the kernel was 7x7 points, k = 3 = (7-1)/2
	k = (kernelSize-1)/2;
	
	Xmax = oldMap->getDimensions(EAST);
	Ymax = oldMap->getDimensions(NORTH);
	Xmin = oldMap->getDimensions(WEST);
	Ymin = oldMap->getDimensions(SOUTH);
	
	//blur vertically
	for(x = Xmin; x<=Xmax; x++)
	{
	tempProb = 0;
	for(int i = k*-1; i<= k; i++)
	{
		tempProb += oldMap->getGridRef(x, Ymin + i);
	}		 
		
	this->updateGridRef(T((tempProb*boxVal)/kernelSize),x, Ymin);
		
	for(y = Ymin + 1; y<= Ymax; y++)
	{
		tempProb += oldMap->getGridRef(x,y + k);
		tempProb -= oldMap->getGridRef(x,y - k - 1);
			
		if(fabs(tempProb) < 0.001)//prevent tiny numbers skewing the result
		tempProb =0;
			
		this->updateGridRef(T((tempProb*boxVal)/kernelSize),x, y);
	}		
	}		
	
	delete oldMap;
	
	oldMap = new GridMap<T>(myMap,dimensions,blockSize, unknown);
	myMap = newBlock();
	this->reset();
	
	tempProb = 0;
	
	//blur horizontally
	for(y = Ymin; y<=Ymax; y++)
	{
	tempProb = 0;
	for(int i = k*-1; i<= k; i++)
	{
		tempProb += oldMap->getGridRef(Xmin + i, y);
	}		
		
	this->updateGridRef(T((tempProb*boxVal)/kernelSize),Xmin, y);
		
	for(x = Xmin + 1; x<= Xmax; x++)
	{
		tempProb += oldMap->getGridRef(x +k,y);
		tempProb -= oldMap->getGridRef(x - k - 1,y);
			
		if(fabs(tempProb) < 0.001)//prevent tiny numbers skewing the result
		tempProb =0;
			
		this->updateGridRef(T((tempProb*boxVal)/kernelSize),x, y);
	}
		
	}
	
	delete oldMap;	
}



//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------

//blurs or smooths the map using a gaussian mask that it first generates
//and then applies to the map

template <class T>
void GridMap<T>::gaussBlur(int kernelSize)
{
	long Xmax, Ymax, Xmin, Ymin; //store the dimensions of the main map 		
	double sum=0;	 
	int x,y, k;//x and y store the current position in the map				
	int n= kernelSize - 1;
	double b[50]; //stores a temp array used in generating the gaussian mask	
	double total;	
	double **mask;
	int i, j = 1;
	double aux[50];
	
	GridMap<T>* oldMap;
	
	//create a new map that uses the same mapblocks as this gridmap
	//so it has the same data without duplicating it	
	oldMap = new GridMap<T>(myMap,dimensions,blockSize, unknown);
	
	//since this map is now stored in oldMap, we create a new map in this object
	//and place the blurred version in it. This way there is only one copy needed,
	//and not two.
	myMap = newBlock();
	reset();
	
	//create the two dimensional array to hold the gaussian mask being applied
	mask = new double*[kernelSize];
	
	for(i=0; i< kernelSize; i++)
	{
	mask[i] = new double[kernelSize];
	}	 
	
	if((kernelSize %2) == 0) kernelSize++; //kernelSize should be an odd number
	
	if(kernelSize < 3) //the smallest possible size of the kernel is 3, since kernel=1 would do nothing
	kernelSize = 3;
	
	//k is half the height of the kernel, eg if the kernel was 7x7 points, k = 3 = (7-1)/2
	k = (kernelSize-1)/2;
	
	//first we must generate a binomial array in b[]
	b[0] = aux[0] = 1;
	for(i = 1; i <= n ; i++) 
	b[i] = aux[i] = 0;
	
	while(j <= n)
	{
	for(i = 1; i <= j; i++)
		b[i] = aux[i-1] + aux[i];
		
	for(i = 1; i <= j; i++) 
		aux[i] = b[i];
		
	j++;
	}
	
	total = b[k] * b[k];
	
	for(i = 0; i < kernelSize; i++)
	{
	for(j = 0; j < kernelSize; j++) 
	{
		mask[i][j] = (b[i] * b[j] * 255) / total;
		sum+= mask[i][j];			
	}
	}
	
	
	for(i = 0; i < kernelSize; i++)
	{
	for(j = 0; j < kernelSize; j++) 
	{
		mask[i][j] /= sum;
	}	
	}
	
	//now that the mask is generated, apply it to the map			
	Xmax = oldMap->getDimensions(EAST);
	Ymax = oldMap->getDimensions(NORTH);
	Xmin = oldMap->getDimensions(WEST);
	Ymin = oldMap->getDimensions(SOUTH);		
	
	for(y = Ymin; y<=Ymax; y++)
	{
	for(x = Xmin; x<= Xmax; x++)
	{
		sum = 0; //initialise sum before applying the mask
			
		//sum up all the values in the map multiplying it by the values in the 
		//kernel
		for(int row = 0; row < kernelSize; row++)
		{
		for(int col=0; col< kernelSize; col++)
		{
			sum += oldMap->getGridRef(x + col - k,y + row -k) * mask[row][col];
		}
		}
			
		this->updateGridRef(T(sum), x,y);	 
	}
	}
	
	//now that a blurred version of this map is stored, delete the old version			
	delete oldMap;
}


//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
/*
//convert a .wld file into a gridmap
template <class T>
bool GridMap<T>::importPointMap(char* fileName, double value, long squareSize)
{
	pointMap *pMap = 0;
	mapLinePtr currentLine = 0;    
	
	//if they enter an incorrect squareSize, do not continue
	if(squareSize <1)
	{
	return false;
	}
	   
	pMap = parseWorldFile(fileName);   
	
	if(pMap == (pointMap*)-1)//if the file doesn't exist
	{	
	return false; 
	}
	
	//now we have our pointMap, which is a list of lines extracted from a .wld world file
	//each line is stored in a mapLine contruct, defined in the pointMap.h file
	
	currentLine = pMap->getLine();
	
	while(currentLine != (mapLinePtr)-1)//if there was a line left in the map
	{
	addLine(currentLine->x1, currentLine->y1, currentLine->x2,currentLine->y2, value, squareSize);			
		
	currentLine = pMap->getLine();
	}	 
	
	delete pMap;
	
	return true;	
}
*/
//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------

//add a straight line from (x1,y1) to (x2, y2) to the map
template <class T>
bool GridMap<T>::addLine(long x1, long y1, long x2, long y2, T value, long squareSize, bool doubleLine)
{
	long temp = 0;
	double slope = 0; //this is the 'm' in the line formula y = mx+c
	long yIntercept = 0; //this is the 'c' in the line formula y = mx+c
	long x=0,y=0;
	long tempX = 0, tempY = 0;
	
	bool noSlope = false;
	long positionDiff = 0;	
	
	//if they enter an incorrect squareSize, do not continue
	if(squareSize <1)
	{
		return false;
	}
	
	//if the line doesn't deviate from a single line of squares, then there's no need to calculate the slope
	//This also prevents division by zero
	if(y2/squareSize != y1/squareSize && x1/squareSize != x2/squareSize)
	{
		slope = double(y2 - y1)/double(x2 - x1);
		noSlope = false;			
	}
	else
	{
		noSlope = true;
	}	 
	
	yIntercept = y1 - long(slope * x1); //if y=mx+c, then c = y - mx
	
	
	if(fabs(y2 - y1) >= fabs(x2 - x1))
	{	//if the distance between the two y's is greater than or equal to the distance
		//between the two x's, we process the line in the y direction.
		//otherwise we process it in the x direction	
		
		if(y2 < y1)//if y2 is below y1, swap the 2 points
		{
			temp = y2;//swap the y's
			y2 = y1;
			y1 = temp;
			
			temp = x2;//swap the x's
			x2 = x1;
			x1 = temp;
		}
		
		//go from y1 to the grid cell containing y2, which is therefore rounded up to the value at the top of the cell
		if(!noSlope)
		{		
			if(x1 > x2) //create an extra line to the right of the current line
			{//because we want to add a line that is one grid space closer to the vertical
				positionDiff = squareSize;
			}
			else
			{
				positionDiff = squareSize * -1;
			}
			
			for(y=y1; y < y2 + (squareSize - (y2 % squareSize)) ; y += squareSize)
			{
				x = long((y - yIntercept)/slope); //if y = mx +c, then x = (y - c)/m

				tempX = ((x < 0) && (x%squareSize != 0)) ? (x/squareSize)-1:x/squareSize;
				tempY = ((y < 0) && (y%squareSize != 0)) ? (y/squareSize)-1:y/squareSize;
				updateGridRef(value, tempX, tempY);

				//updateGridRef(value, x/squareSize, y/squareSize); //set the occupied value of the cell to 1
				if(doubleLine)
				{
					tempX = ((x < 0) && (x%squareSize != 0)) ? ((x+positionDiff)/squareSize)-1:(x+positionDiff)/squareSize;
					updateGridRef(value,tempX, tempY); 
				}
			}			
			
		}
		else
		{		
			//if the line is closer to the right side of the grid cell,create a new line to the right
			if(((x1 + x2)/2)% squareSize >= (squareSize/2))
			{
				positionDiff = squareSize;
			}
			else //otherwise create a new line to the left
			{
				positionDiff = squareSize * -1;
			}
			
			x = x1;
			for(y=y1; y < y2 + (squareSize - (y2 % squareSize)) ; y += squareSize)
			{
				tempX = ((x < 0) && (x%squareSize != 0)) ? (x/squareSize)-1:x/squareSize;
				tempY = ((y < 0) && (y%squareSize != 0)) ? (y/squareSize)-1:y/squareSize;
				updateGridRef(value, tempX, tempY);

				//updateGridRef(value, x/squareSize, y/squareSize); //set the occupied value of the cell to 1
				if(doubleLine)
				{
					tempX = ((x < 0) && (x%squareSize != 0)) ? ((x+positionDiff)/squareSize)-1:(x+positionDiff)/squareSize;
					updateGridRef(value,tempX, tempY); 
				}
			}				
		}
	} 
	else //step in the x direction
	{	
		if(x2 < x1)//if x2 to the left of x1, swap the 2 points
		{
			temp = y2;//swap the y's
			y2 = y1;
			y1 = temp;
			
			temp = x2;//swap the x's
			x2 = x1;
			x1 = temp;
		}
		
		if(!noSlope)
		{
			//LOG<<"\nAdding line X1="<<x1<<",Y1="<<y1<<",X2="<<x2<<", Y2="<<y2;
			//if the line is ascending, create a new line above the current line
			if(y1 > y2)
			{
				positionDiff = squareSize;
			}
			else //otherwise create a new line below the current line
			{
				positionDiff = squareSize * -1;
			}
			//LOG<<"\npositionDiff = "<<positionDiff;
			//LOG<<"\nsquareSize = "<<squareSize;
			//LOG<<"\nSlope = "<<slope<<"\nyIntercept = "<<yIntercept;
			
			
			//go from x1 to the grid cell containing x2, which is therefore rounded up to the value at the right of the cell
			for(x=x1; x < x2 + (squareSize - x2 % squareSize) ; x += squareSize)
			{
				y = long(slope * x) + yIntercept; // y = mx +c
				tempX = ((x < 0) && (x%squareSize != 0)) ? (x/squareSize)-1:x/squareSize;
				tempY = ((y < 0) && (y%squareSize != 0)) ? (y/squareSize)-1:y/squareSize;
				updateGridRef(value, tempX, tempY);

				//updateGridRef(value, x/squareSize, y/squareSize); //set the occupied value of the cell to 1
				if(doubleLine)
				{
					tempY = ((y < 0) && (y%squareSize != 0)) ? ((y + positionDiff)/squareSize)-1:(y + positionDiff)/squareSize;
					updateGridRef(value,tempX, tempY); 								
				}
			}
			
			
		}
		else
		{
			if(((y1 + y2)/2)%squareSize >= (squareSize/2))
			{
				positionDiff = squareSize;
			}
			else
			{
				positionDiff = squareSize * -1;
			}
			
			y = y1;
			//go from x1 to the grid cell containing x2, which is therefore rounded up to the value at the right of the cell
			for(x=x1; x < x2 + (squareSize - x2 % squareSize) ; x += squareSize)
			{
				tempX = ((x < 0) && (x%squareSize != 0)) ? (x/squareSize)-1:x/squareSize;
				tempY = ((y < 0) && (y%squareSize != 0)) ? (y/squareSize)-1:y/squareSize;
				updateGridRef(value, tempX, tempY);

				//updateGridRef(value, x/squareSize, y/squareSize); //set the occupied value of the cell to 1
				if(doubleLine)
				{
					tempY = ((y < 0) && (y%squareSize != 0)) ? ((y + positionDiff)/squareSize)-1:(y + positionDiff)/squareSize;
					updateGridRef(value,tempX, tempY); //set the occupied value of the cell 	
				}
			}
		}
	}	
	
	return true;
}

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------

//this function compares two gridmaps using Baron's correlation coefficient
//it involves getting the average value stored in both maps, the average
//value in the map that results from pixel-by-pixel multiplication of the
//two maps, and the standard deviation of both maps. These figures are then 
//combined using Baron's formula to give a measure of the similarity 
//of the two maps
template <class T>
double GridMap<T>::correlateMap(GridMap<T>* mapToCompare)
{
	double result=0;

	long temp = 0;
	long counter = 0;
	
	//figures for top line of Baron's equation
	double thisAvg=0, map2Avg=0, productAvg=0;
	
	//figures for bottom line of Baron's equation
	double stanDevthis=0, stanDevMap2 = 0;
	
	double sumthis=0, sumMap2=0;
	
	long north=0, south=0, east=0, west=0, x=0,y=0;

	/*
	  //make sure that the two maps are the same dimensions
	  while(this->getDimensions(NORTH) != mapToCompare->getDimensions(NORTH))
	  {
	  //grow the smaller map in the direction it is smaller
	  if(this->getDimensions(NORTH) < mapToCompare->getDimensions(NORTH))
	  {
	  this->updateGridRef(unknown, 0, this->getDimensions(NORTH) + 2);
	  }
	  else
	  {
	  mapToCompare->updateGridRef(unknown,0, mapToCompare->getDimensions(NORTH) + 2);
	  }
	  }
	  while(this->getDimensions(EAST) != mapToCompare->getDimensions(EAST))
	  {
	  //grow the smaller map in the direction it is smaller
	  if(this->getDimensions(EAST) < mapToCompare->getDimensions(EAST))
	  {
	  this->updateGridRef(unknown, this->getDimensions(EAST)+2, 0);
	  }
	  else
	  {
	  mapToCompare->updateGridRef(unknown,mapToCompare->getDimensions(EAST)+2, 0);
	  }
	  }
	  while(this->getDimensions(WEST) != mapToCompare->getDimensions(WEST))
	  {
	  //grow the smaller map in the direction it is smaller
	  if(this->getDimensions(WEST) > mapToCompare->getDimensions(WEST))
	  {
	  this->updateGridRef(unknown, this->getDimensions(WEST)-2, 0);
	  }
	  else
	  {
	  mapToCompare->updateGridRef(unknown, mapToCompare->getDimensions(WEST)-2, 0);
	  }
	  }
	  while(this->getDimensions(SOUTH) != mapToCompare->getDimensions(SOUTH))
	  {
	  //grow the smaller map in the direction it is smaller
	  if(this->getDimensions(SOUTH) > mapToCompare->getDimensions(SOUTH))
	  {
	  this->updateGridRef(unknown, 0, this->getDimensions(SOUTH)-2);
	  }
	  else
	  {
	  mapToCompare->updateGridRef(unknown, 0, mapToCompare->getDimensions(SOUTH)-2);
	  }
	  }
	*/

	
	// compare the area covered by the biggest map
	if(this->getUpdatedDimensions(NORTH) > mapToCompare->getUpdatedDimensions(NORTH))
		north = this->getUpdatedDimensions(NORTH);
	else
		north = mapToCompare->getUpdatedDimensions(NORTH);
	
	if(this->getUpdatedDimensions(EAST) > mapToCompare->getUpdatedDimensions(EAST))
		east = this->getUpdatedDimensions(EAST);
	else
		east = mapToCompare->getUpdatedDimensions(EAST);
	
	if(this->getUpdatedDimensions(SOUTH) < mapToCompare->getUpdatedDimensions(SOUTH))
		south = this->getUpdatedDimensions(SOUTH);
	else
		south = mapToCompare->getUpdatedDimensions(SOUTH);
	
	if(this->getUpdatedDimensions(WEST) < mapToCompare->getUpdatedDimensions(WEST))
		west = this->getUpdatedDimensions(WEST);
	else
		west = mapToCompare->getUpdatedDimensions(WEST);  
	

	counter = 0;
	//get the mean of this map and the map being compared
	for( y = north; y>= south; y--)
	{
		for( x = west; x<= east; x++)
		{
			counter++;

			thisAvg += this->getGridRef(x,y);
			map2Avg += mapToCompare->getGridRef(x,y);
			productAvg += (this->getGridRef(x,y) * mapToCompare->getGridRef(x,y));
		}
	}
	
	//divide the totals by the number of cells in the map
	thisAvg /= counter;//((north - south+1) * (east - west+1));	
	map2Avg /= counter;//((north - south+1) * (east - west+1)); 
	productAvg /= counter;//((north - south+1) * (east - west+1)); 
	
	
	//calculate the standard deviation from the mean for the two maps
	for( y = north; y>= south; y--)
	{
		for( x = west; x<= east; x++)
		{
			sumthis += (this->getGridRef(x,y) - thisAvg) * (this->getGridRef(x,y) - thisAvg);
			sumMap2 += (mapToCompare->getGridRef(x,y) - map2Avg) * (mapToCompare->getGridRef(x,y) - map2Avg);
		}
	}
	sumthis /= (((north - south+1) * (east - west+1)) );
	sumMap2 /= (((north - south+1) * (east - west+1)) );
	
	stanDevthis = sqrt(sumthis);
	stanDevMap2 = sqrt(sumMap2);
	
	result = (productAvg - (thisAvg * map2Avg)) / (stanDevthis * stanDevMap2);
	
	//the function gives values correct to 4 decimal places
	//so cut off all except the last 4 decimal places
	temp = long(result * 10000);
	result = double(temp) / 10000;
	
	return result;
	
	
}

//this function uses Carnegie Mellon's MATCH method to compare two maps.
template <class T>
double GridMap<T>::scoreMap(GridMap<T>* mapToCompare, bool justCompareOccAreas)
{
  double score = 0;

  ofstream outFile;
  //outFile.open("f:\\temp\\gMapReport.txt");
  //outFile<<"\nOpened the file"<<endl;


  long xMax = 0, xMin = 0, yMax = 0, yMin = 0;

  //get the largest possible matching map
  if(this->getUpdatedDimensions(WEST) < mapToCompare->getUpdatedDimensions(WEST))
	xMin = this->getUpdatedDimensions(WEST);
  else
	xMin = mapToCompare->getUpdatedDimensions(WEST);

  if(this->getUpdatedDimensions(EAST) > mapToCompare->getUpdatedDimensions(EAST))
	xMax = this->getUpdatedDimensions(EAST);  
  else
	xMax = mapToCompare->getUpdatedDimensions(EAST);
  
  if(this->getUpdatedDimensions(NORTH) > mapToCompare->getUpdatedDimensions(NORTH))
	yMax = this->getUpdatedDimensions(NORTH);  
  else
	yMax = mapToCompare->getUpdatedDimensions(NORTH);

  if(this->getUpdatedDimensions(SOUTH) < mapToCompare->getUpdatedDimensions(SOUTH))
	yMin = this->getUpdatedDimensions(SOUTH);  
  else
	yMin = mapToCompare->getUpdatedDimensions(SOUTH);

  //long cellCounter = 0;

  T thisMapVal = 0, otherMapVal = 0;

  if(!justCompareOccAreas)
  {	  
	  for(long x = xMin; x <= xMax; x++)
	  {
		  for(long y = yMax; y >= yMin; y--)
		  {	
			  thisMapVal = this->getGridRef(x,y);
			  otherMapVal = mapToCompare->getGridRef(x,y);

			  if(!(thisMapVal == 0.5 && otherMapVal == 0.5))
			  {
				 
				  score += ( fabs(thisMapVal - otherMapVal) * fabs(thisMapVal - otherMapVal));
			  }				
		  }
	  }
  }
  else
  {
	  //only compare the parts of the map that have occupied areas in either of the maps
	  for(long x = xMin; x <= xMax; x++)
	  {
		  for(long y = yMax; y >= yMin; y--)
		  {	
			  thisMapVal = this->getGridRef(x,y);
			  otherMapVal = mapToCompare->getGridRef(x,y);

			  if(thisMapVal > 0.5 || otherMapVal > 0.5)
			  {
				  score += ( fabs(thisMapVal - otherMapVal) * fabs(thisMapVal - otherMapVal));
			  }
			  			
		  }
	  }
	  
  }

  //outFile<<"\nOut of a total of "<<cellCounter<<" cells, we get a score of "<<score<<endl
//	  <<" which gives an overall score of "<<1 - (score / cellCounter)<<endl;
 
  if(score < 0.00001)
  {
	  score = 0;
  }
    
  //outFile<<score<<"\nTHis is subtracted from one to give "<<1 - score<<endl;
  //outFile.close();


  return score;
}
/*
//this function uses Carnegie Mellon's MATCH method to compare two maps.
template <class T>
double GridMap<T>::scoreMap(GridMap<T>* mapToCompare)
{
  double score = 0;

  ofstream outFile;
  //outFile.open("f:\\temp\\gMapReport.txt");
  //outFile<<"\nOpened the file"<<endl;


  long xMax = 0, xMin = 0, yMax = 0, yMin = 0;

  //get the largest possible matching map
  if(this->getUpdatedDimensions(WEST) < mapToCompare->getUpdatedDimensions(WEST))
	xMin = this->getUpdatedDimensions(WEST);
  else
	xMin = mapToCompare->getUpdatedDimensions(WEST);

  if(this->getUpdatedDimensions(EAST) > mapToCompare->getUpdatedDimensions(EAST))
	xMax = this->getUpdatedDimensions(EAST);  
  else
	xMax = mapToCompare->getUpdatedDimensions(EAST);
  
  if(this->getUpdatedDimensions(NORTH) > mapToCompare->getUpdatedDimensions(NORTH))
	yMax = this->getUpdatedDimensions(NORTH);  
  else
	yMax = mapToCompare->getUpdatedDimensions(NORTH);

  if(this->getUpdatedDimensions(SOUTH) < mapToCompare->getUpdatedDimensions(SOUTH))
	yMin = this->getUpdatedDimensions(SOUTH);  
  else
	yMin = mapToCompare->getUpdatedDimensions(SOUTH);

  long cellCounter = 0;

  double naturalLogOf2 = log(2);

  //outFile<<"\nAbout to match maps"<<endl;

  for(long x = xMin; x <= xMax; x++)
  {
	for(long y = yMax; y >= yMin; y--)
	{	
		if(!(this->getGridRef(x,y) == 0.5 || mapToCompare->getGridRef(x,y) == 0.5))
		{
			cellCounter ++;
		}
		//outFile<<"Cell number "<<cellCounter<<": score += 1 + "
		//	<<"log("<<(this->getGridRef(x,y) * mapToCompare->getGridRef(x,y))<<" + "
		//	<<( (1-this->getGridRef(x,y)) * (1-mapToCompare->getGridRef(x,y)) ) <<" )/"<<naturalLogOf2
		//	<<" = 1 + "<<log((this->getGridRef(x,y) * mapToCompare->getGridRef(x,y)) + 
		//	( (1-this->getGridRef(x,y)) * (1-mapToCompare->getGridRef(x,y)) )  )
		//	<<"/"<<naturalLogOf2<<" = ";
			

		score += 1 + 
			log((this->getGridRef(x,y) * mapToCompare->getGridRef(x,y)) + 
			( (1-this->getGridRef(x,y)) * (1-mapToCompare->getGridRef(x,y)) )  )/naturalLogOf2;
		//outFile<<score<<endl;
	}
  }
  
 // outFile<<"\nFinished Matching Maps"<<endl;

  if(cellCounter != 0)
  {
	  score /= cellCounter;
  }
  else
  {
	  score = 0;
  }
 // outFile.close();


  return score;
}


  */



//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------

//this function grows the occupied areas of the map by a given radius. 
//the most likely application of this is in Path Planning, the the occupies
//areas are grown by half the radius of the robot so that the navigation 
//algorithms only have to worry about navigating with a point, rather than 
//having to calculate if a robot of a given shape could fit between two obstacles
//The functions only operates on grid cells with values between lowerBound and upperBound
//For each of these cells, it changes it to valueToUpdateTo, and does the same to all other 
//cells within a distance of 'radius' millimetres around it
template <class T>
bool GridMap<T>::growOccArea(long radius, T lowerBound, T upperBound, long squaresize)
{
	
//	long oldMapDimensions[4]; 
	long left = 0, right = 0, top = 0, bottom = 0;
	long roundUp = 0;
	T tempVal;
	
	long xDiff = 0, yDiff = 0;
	
	if(radius <= 0 || squaresize == 0)
	{
		return false;
	}
	
	long minX=0,maxX=0,minY=0,maxY=0;
	maxY = getUpdatedDimensions(NORTH);
	minY = getUpdatedDimensions(SOUTH);
	maxX  = getUpdatedDimensions(EAST);
	minX  = getUpdatedDimensions(WEST);
	
	//first, create a new GridMap, copy our map into it
	GridMap<T> *oldMap;
	oldMap = new GridMap<T>(blockSize, 1, unknown);
	oldMap->clone(this);	

	//now we start processing, reading in values from the oldMap, and inserting them	
	//into our new map
	for(long x = minX * squaresize + squaresize/2; 
			x <= maxX *squaresize + squaresize/2; 
			x+=squaresize)
	{
		for(long y = maxY * squaresize + squaresize/2
			; y >= minY*squaresize + squaresize/2; y-=squaresize)
		{
			tempVal = oldMap->getGridRef(x/squaresize,y/squaresize);
			
			//if the value in this cell is considered to be occupied
			//then update all the cells around it within a radius of 'radius'
			//with the same value as this cell
			if(tempVal >= lowerBound && tempVal <= upperBound)
			{
				roundUp = - (radius%squaresize) + (squaresize/2);
				
				left   = (x - (squaresize/2) - radius) - roundUp;
				right  = (x + (squaresize/2) + radius) + roundUp;
				top    = (y + (squaresize/2) + radius) + roundUp;
				bottom = (y - (squaresize/2) - radius) - roundUp;
				
				for(long X2 = left;X2 <= right; X2+= squaresize)
				{
					for(long Y2 = top; Y2 >= bottom; Y2-= squaresize)
					{
						if(getGridRef(X2/squaresize,Y2/squaresize) < tempVal)
						{							
							if(X2 == x)//if the squares are above or below each other
							{
								xDiff = 0;
							}
							//pick the smallest distance from one edge of a square to the other
							else if(X2 < x)
							{
								xDiff = long(fabs((X2 + squaresize/2) - (x - squaresize/2)));
							}
							else
							{
								xDiff = long(fabs((X2 - squaresize/2) - (x + squaresize/2)));
							}
							
							if(Y2 == y) //if the squares are across from one another
							{
								yDiff = 0;
							}
							else if(Y2 < y)
							{
								yDiff = long(fabs((Y2 + squaresize/2) - (y - squaresize/2)));
							}
							else
							{	
								yDiff = long(fabs((Y2 - squaresize/2) - (y + squaresize/2)));
							}					
							
							//check the hypotenuse, if it's less than the radius of the circle,
							//update the point
							if(sqrt(xDiff*xDiff + yDiff*yDiff) <= radius)
							{
								updateGridRef(tempVal,X2/squaresize,Y2/squaresize);
							}
						}
					}
				}				
			}
			else
			{
				updateGridRef(tempVal,x/squaresize,y/squaresize);
			}
		}
	}
	
	delete oldMap;	

	minX = SosUtil::minVal(minX,getUpdatedDimensions(WEST));
	maxX = SosUtil::maxVal(maxX,getUpdatedDimensions(EAST));
	minY = SosUtil::minVal(minY,getUpdatedDimensions(SOUTH));
	maxY = SosUtil::maxVal(maxY,getUpdatedDimensions(NORTH));
	setDimensions(minX,maxY,maxX,minY);
	
	return true;	
}


template <class T>
double GridMap<T>::getGridRefFromTopLeftView(long inputX, long inputY)
{
	if(!topLeftSet)
	{
		topLeftX = getUpdatedDimensions(WEST);
		topLeftY = getUpdatedDimensions(NORTH);
		topLeftSet = true;
	}

	//LOG<<"\ngetGridRefFromTopLeftView("<<inputX<<","<<inputY<<") returning (";

	inputX += topLeftX;
	inputY = topLeftY - inputY;

	//LOG<<inputX<<","<<inputY<<")";

	return getGridRef(inputX, inputY);

}

/*
template <class T>
double GridMap<T>::getGridRefFromView(long inputX, long inputY)
{
	long leftMost = getUpdatedDimensions(WEST);
	long bottomMost = getUpdatedDimensions(SOUTH);

	inputX += leftMost;
	inputY += bottomMost;

	return getGridRef(inputX, inputY);

}
*/
template <class T>
double GridMap<T>::getGridRefFromView(long inputX, long inputY,long viewHeight, long viewWidth)
{	/*
	double actualWidth = getMapWidth();
	double actualHeight = getMapHeight();

	double leftX = (actualWidth/double(viewWidth)) * inputX;
	double rightX = (actualWidth/double(viewWidth)) * (inputX+1);

	double topY = (actualHeight/viewHeight) * (inputY+1);
	double bottomY = (actualHeight/viewHeight) * (inputY);

	double perc = 0, percSum = 0;

	double retSum = 0;

	double upperY = 0, lowerY = 0, smallX = 0, bigX = 0;

	for(long x = (long)SosUtil::roundDown(leftX); x<SosUtil::roundUp(rightX); x++)
	{
		for(long y = (long)SosUtil::roundDown(topY); y >= SosUtil::roundDown(bottomY); y--)
		{
			if(SosUtil::between(topY,y, y+1))
				upperY = topY;
			else
				upperY = y+1;

			if(SosUtil::between(bottomY,y,y+1))
				lowerY = bottomY;
			else
				lowerY = y;

			if(SosUtil::between(leftX,x,x+1))
				smallX = leftX;
			else
				smallX = x;

			if(SosUtil::between(rightX,x,x+1))
				bigX = rightX;
			else
				bigX = x+1;

			perc = (double)(upperY - lowerY) * (double)(bigX - smallX);

			percSum += perc;

			retSum += perc * getGridRef(x,y);

		}
	}

	return retSum/percSum;
	*/

	return getGridRefFromView(inputX, inputY,viewHeight,viewWidth,getUpdatedDimensions(WEST),
		getUpdatedDimensions(NORTH),getUpdatedDimensions(EAST),getUpdatedDimensions(SOUTH));

}

template <class T>
double GridMap<T>::getGridRefFromView(long inputX, long inputY,long viewHeight, long viewWidth
			,long westBorder, long northBorder,long eastBorder,long southBorder)
{
	double actualWidth = eastBorder - westBorder +1;
	double actualHeight = northBorder - southBorder +1;

	double leftX = (actualWidth/double(viewWidth)) * inputX;
	double rightX = (actualWidth/double(viewWidth)) * (inputX+1);

	double topY = (actualHeight/viewHeight) * (inputY+1);
	double bottomY = (actualHeight/viewHeight) * (inputY);

	double perc = 0, percSum = 0;

	double retSum = 0;

	double upperY = 0, lowerY = 0, smallX = 0, bigX = 0;

	for(long x = (long)SosUtil::roundDown(leftX); x<SosUtil::roundUp(rightX); x++)
	{
		for(long y = (long)SosUtil::roundDown(topY); y >= SosUtil::roundDown(bottomY); y--)
		{
			if(SosUtil::between(topY,(double)y, (double)(y+1)))
				upperY = topY;
			else
				upperY = y+1;

			if(SosUtil::between(bottomY,(double)y, (double)(y+1)))
				lowerY = bottomY;
			else
				lowerY = y;

			if(SosUtil::between(leftX,(double)x,(double)(x+1)))
				smallX = leftX;
			else
				smallX = x;

			if(SosUtil::between(rightX,(double)x,(double)(x+1)))
				bigX = rightX;
			else
				bigX = x+1;

			perc = (double)(upperY - lowerY) * (double)(bigX - smallX);

			percSum += perc;

			retSum += perc * getGridRef(x,y);

		}
	}

	return retSum/percSum;
}

template <class T>
double GridMap<T>::getGridRefFromTopLeftView(long inputX, long inputY,long viewHeight, long viewWidth)
{
	double actualWidth = getMapWidth();
	double actualHeight = getMapHeight();

	return getGridRefFromTopLeftView(inputX, inputY,viewHeight, viewWidth, getMapWidth(),getMapHeight());

}

template <class T>
double GridMap<T>::getGridRefFromTopLeftView(long inputX, long inputY,long viewHeight, long viewWidth
			,long mapWidth, long mapHeight)
{
	double actualWidth = mapWidth;
	double actualHeight = mapHeight;

	//LOG<<"\ngetGridRefFromTopLeftView(...) entry";
	//LOG<<"\ninputX = "<<inputX<<", inputY = "<<inputY;

	double leftX = (actualWidth/double(viewWidth)) * inputX;
	double rightX = (actualWidth/double(viewWidth)) * (inputX+1);

	double topY = (actualHeight/viewHeight) * (inputY+1);
	double bottomY = (actualHeight/viewHeight) * (inputY);

	//LOG<<"\ntopY = ("<<actualHeight<<"/"<<viewHeight<<") * ("<<inputY+1<<") = "<<topY;

	double perc = 0, percSum = 0;

	double retSum = 0;

	double upperY = 0, lowerY = 0, smallX = 0, bigX = 0;

	for(long x = (long)SosUtil::roundDown(leftX); x<SosUtil::roundUp(rightX); x++)
	{
		for(long y = (long)SosUtil::roundDown(topY); y >= SosUtil::roundDown(bottomY); y--)
		{
			if(SosUtil::between(topY,(double)y, (double)(y+1)))
				upperY = topY;
			else
				upperY = y+1;

			if(SosUtil::between(bottomY,(double)y, (double)(y+1)))
				lowerY = bottomY;
			else
				lowerY = y;

			if(SosUtil::between(leftX,(double)x,(double)(x+1)))
				smallX = leftX;
			else
				smallX = x;

			if(SosUtil::between(rightX,(double)x,(double)(x+1)))
				bigX = rightX;
			else
				bigX = x+1;

			perc = (double)(upperY - lowerY) * (double)(bigX - smallX);

			percSum += perc;

			retSum += perc * getGridRefFromTopLeftView(x,y);

		}
	}

	return retSum/percSum;

}

template <class T>
void GridMap<T>::setTopLeftPos(long x, long y)
{
	topLeftX = x;
	topLeftY = y;
	topLeftSet = true;
}

template <class T>
bool GridMap<T>:: addLineFromView(long x1, long y1, long x2, long y2, T value, long squareSize, 
			long viewHeight, long viewWidth,bool doubleLine)
{
	long leftMost = getUpdatedDimensions(WEST);
	long bottomMost = getUpdatedDimensions(SOUTH);

	//translate the points
	x1 += leftMost*squareSize;
	y1 += bottomMost*squareSize;
	x2 += leftMost*squareSize;
	y2 += bottomMost*squareSize;

	return addLine(x1,y1,x2,y2,value,squareSize,doubleLine);
}

template <class T>
void GridMap<T>:: resize(float degree)
{/*
	long west=0,east=0,north=0,south = 0;
	float val = 0;
	getAllUpdatedDimensions(west,north,east,south);

	west  = (long)((float)west  * degree);
	east  = (long)((float)east  * degree);
	south = (long)((float)south * degree);
	north = (long)((float)north * degree);

	GridMap<T> newMap(blockSize,0,unknown);

	long height = north - south + 1;
	long width = east - west + 1;

	for(long x = west; x<=east; x++)
	{
		for(long y = south; y<= north; y++)
		{
			val = getGridRefFromView(x,y,height,width);
			newMap.updateGridRef(val,x,y);
		}

	}
*/
}



//#endif
