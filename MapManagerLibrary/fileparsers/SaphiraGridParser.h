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

#ifndef SAPHIRA_GRID_PARSER
#define SAPHIRA_GRID_PARSER

#include "SosList.h"
#include "SosUtil.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../logger/Logger.h"

class SaphiraGridParser
{
	public:
	SaphiraGridParser()
	{
		GET_FILE_LOG
		_listPoints.setModeQueue();		
		minX = maxX =  minY = maxY = 0;
	}

	void getDimensions(long& west, long& north, long& east, long& south)
	{
		west = minX;
		east = maxX;
		south = minY;
		north = maxY;
	}
	
	
	bool parseMapFile(char* fileName)
	{
		LOG<<"In parseWldFile";
		char c = 0;
	//	SosUtil::cos(1.4);//take out
		if(fileName == 0)
		{
			LOG<<"filename is null, returning false";
			return false;
		}
		PointXYLong pt;
		minX = maxX =  minY = maxY = 0;
		long numPoints =0;//numPoints is not used
		long resolution = 0;

		_listPoints.clear();


		FILE* wldFile = 0;
		
		char token[50] = {0};
		//2D-Map
		bool foundId = false;
		
		wldFile = fopen(fileName,"r"); //open the file for reading
		if(wldFile == NULL)
		{
			LOG<<"Could not open wldFile "<<fileName;
			return false;
		}
		
		if(feof(wldFile)) //if we're at the end of the file, return false
		{
			LOG<<"File is empty, returning false";
			return false;
		}
		
		long temp = 0;	
		bool inData = false;
		
		LOG<<"About to go into While loop";
		while((c = fgetc(wldFile)) != EOF)
		{
			if ( c==' ' || c =='\n' || c == '\t')
			{
				continue;
			}
			else
			{
				
				if (c == ';' || c == '#' || c == 'C' || c == 'c')    //Skip comments and lines starting with Cairn
				{
					do
					c = fgetc(wldFile);
					while (c != '\n' && c != EOF);
					ungetc(c,wldFile);
				}
				else
				{	
					ungetc(c, wldFile);
					fscanf(wldFile,"%s",&token);

					//the first non-comment should be "2D-Map"
					if(!foundId && stricmp(token,"2D-Map") != 0)
					{
						_listPoints.clear();
						LOG<<"Returning false 1";
						return false;
					}
					else if(!foundId)
					{
						foundId = true;
						continue;
					}
					
					//if the token is numeric, it's the start of a line, so read the next 3 tokens
					if(inData && SosUtil::is_numeric(token))
					{	
						eatSpace(wldFile);
						pt.x = atol(token);
						
						if(!SosUtil::readNum(wldFile,pt.y))
						{
							_listPoints.clear();
							LOG<<"Returning false 2";
							return false;
						}
						
						_listPoints.push(pt);
					}
					else
					{
						if(stricmp(token,"MinPos:") == 0)
						{
							eatSpace(wldFile);

							if(!SosUtil::readNum(wldFile,minX))
							{
								_listPoints.clear();
								LOG<<"Returning false 3";
								return false;
							}
														
							eatSpace(wldFile);

							if(!SosUtil::readNum(wldFile,minY))
							{
								_listPoints.clear();
								LOG<<"Returning false 4";
								return false;
							}
						}
						else if(stricmp(token,"MaxPos:") == 0)
						{
							eatSpace(wldFile);

							if(!SosUtil::readNum(wldFile,maxX))
							{
								_listPoints.clear();
								LOG<<"Returning false 5";
								return false;
							}
														
							eatSpace(wldFile);

							if(!SosUtil::readNum(wldFile,maxY))
							{
								_listPoints.clear();
								LOG<<"Returning false 6";
								return false;
							}
						}
						else if(stricmp(token,"NumPoints:") == 0)
						{
							eatSpace(wldFile);
							if(!SosUtil::readNum(wldFile,numPoints))
							{
								_listPoints.clear();
								LOG<<"Returning false 7";
								return false;
							}
						}
						else if(stricmp(token,"DATA") == 0)
						{
							inData = true;
						}
						else if(stricmp(token,"resolution:") == 0)
						{
							eatSpace(wldFile);
							if(!SosUtil::readNum(wldFile,resolution))
							{
								_listPoints.clear();
								LOG<<"Returning false 8";
								return false;
							}
						}
						else
						{
							LOG<<"Failed on token: "<<token;
							_listPoints.clear();
							return false;
							//return false;//if the token in non-numeric and not one of the words above, it's a parse error
						}
					}									
				}				
			}			
		}
		
		fclose(wldFile);
		
		return true;
	}
	
	void resetIterator()
	{
		_listPoints.resetIterator();
	}
	
	bool getNext(PointXYLong& line)
	{
		return _listPoints.readNext(line);
	}


private:
	void eatSpace(FILE* file)
	{
		char c = 0;
		do 
		{
			c = fgetc(file);
		}		
		while (c == ' ' || c == '\n' || c == ',');
		
		ungetc(c, file);
	}

	List<PointXYLong> _listPoints;
	long minX , maxX , minY ,maxY;
	DEF_LOG


};




#endif