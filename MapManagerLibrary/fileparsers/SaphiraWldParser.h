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

#ifndef SAPHIRA_WLD_PARSER
#define SAPHIRA_WLD_PARSER

#include "SosList.h"
#include "SosUtil.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../logger/Logger.h"

#define DEG_TO_RAD (3.14159265358979323846 / 180.0)
#define RAD_TO_DEG (180.0 / 3.14159265358979323846)

class SaphiraWldParser
{
public:
	SaphiraWldParser()
	{
		GET_FILE_LOG
		_listLines.setModeQueue();
		_listPushedPositions.setModeStack();
		_width = _height = 0;
		_origin.x = _origin.y = _robotPos.x = _robotPos.y = 0;
		_hasRobotPos = 0;
		_robotPos.x = _robotPos.y = 0;
		_robotPos.value = 0;
		_hasRobotPos = 0;
	}
	
	
	bool parseWldFile(char* fileName)
	{
		LOG<<"In parseWldFile";
		char c = 0;
		
		if(fileName == 0)
		{
			LOG<<"filename is null, returning false";
			return false;
		}
		LineXYLong line, line2;
		PointXYZ pushPoint, tempPoint;
		FILE* wldFile = 0;
		
		char token[50] = {0};
		
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
		
		pushPoint.x = 0;
		pushPoint.y = 0;
		pushPoint.value = 0;
		
		long temp = 0;
		bool trySecondFileType = false;
		
		_listPushedPositions.push(pushPoint);
		
		LOG<<"About to go into While loop";
		while((c = fgetc(wldFile)) != EOF)
		{
			if ( c==' ' || c =='\n' || c == '\t')
			{
				continue;
			}
			else
			{				
				if (c == ';' || c == '#')    //Skip comments
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
					
					//if the token is numeric, it's the start of a line, so read the next 3 tokens
					if(is_numeric(token))
					{
						_listPushedPositions.readHead(pushPoint);
						
						line.pt1.x = atol(token) + pushPoint.x;
						eatSpace(wldFile);
						
						fscanf(wldFile,"%s",&token);
						if(!is_numeric(token))
						{
							LOG<<"Failed on token: "<<token;
							trySecondFileType = true;
							break;
							//	return false;//parse error
						}
						line.pt1.y = atol(token) + pushPoint.y;
						
						eatSpace(wldFile);
						fscanf(wldFile,"%s",&token);
						if(!is_numeric(token))
						{
							LOG<<"Failed on token: "<<token;
							trySecondFileType = true;
							break;
							//return false;//parse error
						}
						line.pt2.x = atol(token) + pushPoint.x;
						
						eatSpace(wldFile);
						fscanf(wldFile,"%s",&token);
						if(!is_numeric(token))
						{
							LOG<<"Failed on token: "<<token;
							trySecondFileType = true;
							break;
							//return false;//parse error
						}
						line.pt2.y = atol(token) + pushPoint.y;
						
						line2.pt1.x = long((line.pt1.x * SosUtil::cos(pushPoint.value)) + (line.pt1.y * SosUtil::sin(pushPoint.value))); 
						line2.pt1.y = long((line.pt1.y * SosUtil::cos(pushPoint.value)) + (line.pt1.x * SosUtil::sin(pushPoint.value))); 
						
						line2.pt2.x = long((line.pt2.x * SosUtil::cos(pushPoint.value)) + (line.pt2.y * SosUtil::sin(pushPoint.value))); 
						line2.pt2.y = long((line.pt2.y * SosUtil::cos(pushPoint.value)) + (line.pt2.x * SosUtil::sin(pushPoint.value))); 
						
						_listLines.push(line2);
					}
					else
					{
						if(strcmp(token,"origin") == 0)
						{
							eatSpace(wldFile);
							fscanf(wldFile,"%s",&token);
							if(!is_numeric(token))
							{
								LOG<<"Failed on token: "<<token;
								trySecondFileType = true;
								break;
								//return false;//parse error
							}
							
							_origin.x = atol(token);
							
							eatSpace(wldFile);
							fscanf(wldFile,"%s",&token);
							if(!is_numeric(token))
							{
								LOG<<"Failed on token: "<<token;
								trySecondFileType = true;
								break;
								//return false;//parse error
							}
							
							_origin.y = atol(token);					
							
						}
						else if(strcmp(token,"width") == 0)
						{
							eatSpace(wldFile);
							fscanf(wldFile,"%s",&token);
							if(!is_numeric(token))
							{
								LOG<<"Failed on token: "<<token;
								trySecondFileType = true;
								break;
								//return false;//parse error
							}
							
							_width = atol(token);
						}
						else if(strcmp(token,"height") == 0)
						{
							eatSpace(wldFile);
							fscanf(wldFile,"%s",&token);
							if(!is_numeric(token))
							{
								LOG<<"Failed on token: "<<token;
								trySecondFileType = true;
								break;
								//return false;//parse error
							}
							
							_height = atol(token);
						}
						else if(strcmp(token,"position") == 0)
						{
							eatSpace(wldFile);
							fscanf(wldFile,"%s",&token);
							if(!is_numeric(token))
							{
								LOG<<"Failed on token: "<<token;
								trySecondFileType = true;
								break;
							}
							
							_robotPos.x = atol(token);
							
							eatSpace(wldFile);
							fscanf(wldFile,"%s",&token);
							if(!is_numeric(token))
							{
								LOG<<"Failed on token: "<<token;
								trySecondFileType = true;
								break;
								//return false;//parse error
							}
							
							_robotPos.y = atol(token);
							
							eatSpace(wldFile);
							fscanf(wldFile,"%s",&token);
							if(!is_numeric(token))
							{
								LOG<<"Failed on token: "<<token;
								trySecondFileType = true;
								break;
								//return false;//parse error
							}
							
							_robotPos.value= (float)atol(token);
							_hasRobotPos = true;
						}
						else if(strcmp(token, "push") == 0)
						{
							_listPushedPositions.readHead(tempPoint);
							eatSpace(wldFile);
							fscanf(wldFile,"%s",&token);
							if(!is_numeric(token))
							{
								LOG<<"Failed on token: "<<token;
								trySecondFileType = true;
								break;
								//return false;//parse error
							}
							
							pushPoint.x = atol(token) + tempPoint.x;
							
							eatSpace(wldFile);
							fscanf(wldFile,"%s",&token);
							if(!is_numeric(token))
							{
								LOG<<"Failed on token: "<<token;
								trySecondFileType = true;
								break;
								//return false;//parse error
							}
							
							pushPoint.y = atol(token)+ tempPoint.y;
							
							eatSpace(wldFile);
							fscanf(wldFile,"%s",&token);
							if(!is_numeric(token))
							{
								LOG<<"Failed on token: "<<token;
								trySecondFileType = true;
								break;
								//return false;//parse error
							}
							
							pushPoint.value = (float)atof(token) + tempPoint.value;
							
							_listPushedPositions.push(pushPoint);
						}
						else if(strcmp(token, "pop") == 0)
						{
							_listPushedPositions.popHead(pushPoint);
						}
						else
						{
							LOG<<"Failed on token: "<<token;
							trySecondFileType = true;
							break;
							//return false;//if the token in non-numeric and not one of the words above, it's a parse error
						}
						//{
							
						//	LOG<<"Failed on token: "<<token;
						//	trySecondFileType = true;
						//	break;
						//}
					}									
				}				
			}			
		}
		
		fclose(wldFile);
		
		if(!trySecondFileType)//if this is true, then there was a parse error
		{
			LOG<<"trySecondFileType = false, returning true";
			return true;
		}
		{
			LOG<<"trySecondFileType = true, trying second file type";
		}
		
		_hasRobotPos = false;
		return parseWldFile2(fileName);
	}
	
	
	void getMapInfo(long& width, long& height, PointXYLong& origin, PointXYZ& robotPos)
	{
		width = _width;
		height = _height;
		origin = _origin;
		robotPos = _robotPos;
	}
	
	void resetIterator()
	{
		_listLines.resetIterator();
	}
	
	bool getNext(LineXYLong& line)
	{
		return _listLines.readNext(line);
	}

	PointXYZ getRobotPosition()
	{
		return _robotPos;
	}
	
	bool hasRobotPosition()
	{
		return _hasRobotPos;
	}
	
	
private:
	
	bool parseWldFile2(char* fileName)
	{
		LOG<<"Trying second Saphira file type";
		LineXYLong line1, line2;
		List<LineXYLong> lines, doors;

		bool badFile = false;
		
		FILE *fmap = 0;
		char c;   
		char name_artifact[40];
		int  id_artifact;
		int  x_artifact, y_artifact, th_artifact, l_artifact, w_artifact; 
		int xw2_artifact, yw2_artifact, xw1_artifact, yw1_artifact; // coordinates of walls
		int x1line, y1line, x2line, y2line; // cordinates of lines
		int xd1line,yd1line, xd2line, yd2line; // coordinates of doors
		int fin =0;
		
		int temp1, temp2, dist1, dist2;  
		
		fmap  = fopen(fileName, "r");
		
		if (feof(fmap))
			exit(0);
		
		while ((c = fgetc(fmap)) != EOF)
		{
			if ( c==' ' || c =='\n')
			{
				continue;
			}
			else if (c == ';')    //  Comments 
			{
				do
				c = fgetc(fmap);
				while (c != '\n' && c != EOF);
				ungetc(c,fmap);
			} 
			else   // Artifacts 
			{
				ungetc(c,fmap);
				fscanf(fmap,"%s", &name_artifact);
				
				if (!strcmp(name_artifact, "DOOR") 
					|| !strcmp(name_artifact, "JUNCTION") 
					|| !strcmp(name_artifact, "CORRIDOR"))  // artifacts with id 
				{
					eatSpace(fmap);
					fscanf(fmap,"(%i)", &id_artifact);
				}
				
				eatSpace(fmap); 
				
				if(!SosUtil::readNum(fmap,x_artifact))
				{
					badFile = true;
					break;
				}
				eatSpace(fmap); 
				if(!SosUtil::readNum(fmap,y_artifact))
				{
					badFile = true;
					break;
				}
				eatSpace(fmap); 
				if(!SosUtil::readNum(fmap,th_artifact))
				{
					badFile = true;
					break;
				}
				eatSpace(fmap); 

				//fscanf(fmap,"%i", &x_artifact); 
				//eatSpace(fmap); 
				
				//fscanf(fmap,"%i", &y_artifact);
				//eatSpace(fmap); 
				
				//fscanf(fmap,"%i", &th_artifact);
				//eatSpace(fmap);
				
				if (!strcmp(name_artifact, "DOOR") || !strcmp(name_artifact, "JUNCTION"))  // artifacts with width only 
				{
					if(!SosUtil::readNum(fmap,w_artifact))
					{
						badFile = true;
						break;
					}
					//fscanf(fmap,"%i", &w_artifact);
				}
				else 
				{
					if (!strcmp(name_artifact, "WALL"))    // artifact with length only  
					{
						if(!SosUtil::readNum(fmap,l_artifact))
						{
							badFile = true;
							break;
						}
						//fscanf(fmap,"%i", &l_artifact);
					}
					else
					{
						if(!SosUtil::readNum(fmap,l_artifact))
						{
							badFile = true;
							break;
						}
						//fscanf(fmap,"%i", &l_artifact);
						eatSpace(fmap);
						if(!SosUtil::readNum(fmap,w_artifact))
						{
							badFile = true;
							break;
						}
						
						//fscanf(fmap,"%i ", &w_artifact);
						
					} // wall 
				} // door junction 
				
				// transform ARTIFACTS into lines 
				if ( !strcmp(name_artifact, "CORRIDOR") ) 
				{
					// transform  corridor into 2 walls 
					
					xw1_artifact = int(x_artifact + w_artifact * sin(th_artifact * DEG_TO_RAD) / 2);
					yw1_artifact = int(y_artifact - w_artifact * cos(th_artifact* DEG_TO_RAD) / 2);
					
					// transform wall1 into line 
					
					x1line = int(xw1_artifact + l_artifact * cos(th_artifact * DEG_TO_RAD) / 2);
					y1line =  int(yw1_artifact + l_artifact * sin(th_artifact * DEG_TO_RAD) / 2);
					
					line1.pt1.x = x1line;
					line1.pt1.y = y1line;
					
					//    fprintf(fline, "%i %i ", x1line, y1line);
					
					x2line = int(xw1_artifact - l_artifact * cos(th_artifact * DEG_TO_RAD) / 2);
					y2line =  int(yw1_artifact - l_artifact * sin(th_artifact * DEG_TO_RAD) / 2);
					
					line1.pt2.x = x2line;
					line1.pt2.y = y2line;
					lines.push(line1);
					//      fprintf(fline,  "%i %i\n", x2line, y2line);
					
					// transform  corridor into 2 walls 
					xw2_artifact = int(x_artifact - w_artifact * sin(th_artifact * DEG_TO_RAD) / 2);
					yw2_artifact = int(y_artifact + w_artifact * cos(th_artifact * DEG_TO_RAD) / 2);
					
					// transform wall1 into line 
					x1line = int(xw2_artifact + l_artifact * cos(th_artifact * DEG_TO_RAD) / 2);
					y1line =  int(yw2_artifact + l_artifact * sin(th_artifact * DEG_TO_RAD) / 2);
					
					line1.pt1.x = x1line;
					line1.pt1.y = y1line;
					//fprintf(fline, "%i %i ", x1line, y1line);
					
					// transform wall2 into line 
					x2line = int(xw2_artifact - l_artifact * cos(th_artifact * DEG_TO_RAD) / 2);
					y2line =  int(yw2_artifact - l_artifact * sin(th_artifact * DEG_TO_RAD) / 2);
					
					line1.pt2.x = x2line;
					line1.pt2.y = y2line;
					lines.push(line1);
					//fprintf(fline,  "%i %i\n", x2line, y2line);
					
				} // end  corridor 
				
				if ( !strcmp(name_artifact, "WALL") ) 
				{
					// transform wall into line 
					x1line = int(x_artifact + l_artifact * cos(th_artifact * DEG_TO_RAD) / 2);
					y1line =  int(y_artifact + l_artifact * sin(th_artifact * DEG_TO_RAD) / 2);
					
					line1.pt1.x = x1line;
					line1.pt1.y = y1line;
					//fprintf(fline, "%i %i ", x1line, y1line);
					
					x2line = int(x_artifact - l_artifact * cos(th_artifact * DEG_TO_RAD) / 2);
					y2line =  int(y_artifact - l_artifact * sin(th_artifact * DEG_TO_RAD) / 2);
					line1.pt2.x = x2line;
					line1.pt2.y = y2line;
					lines.push(line1);
					//fprintf(fline,  "%i %i\n", x2line, y2line);
					
				} // end wall 
				
				if ( !strcmp(name_artifact, "DOOR") || !strcmp(name_artifact, "JUNCTION") ) 
				{ 
					// transform door, junction into lines 
					// keep lines in file fdoor remember th for door and junction is the normal vector
					//unlike walls cos and sin are inverted  
					
					
					x1line = int(x_artifact + w_artifact * sin(th_artifact * DEG_TO_RAD) / 2);    
					y1line =  int(y_artifact + w_artifact * cos(th_artifact * DEG_TO_RAD) / 2);
					line1.pt1.x = x1line;
					line1.pt1.y = y1line;
					//fprintf(fdoor, "%i %i ", x1line, y1line);
					
					x2line = int(x_artifact - w_artifact * sin(th_artifact * DEG_TO_RAD) / 2);
					y2line =  int(y_artifact - w_artifact * cos(th_artifact * DEG_TO_RAD) / 2);
					line1.pt2.x = x2line;
					line1.pt2.y = y2line;
					doors.push(line1);
					//fprintf(fdoor,  "%i %i\n", x2line, y2line);
					
				} // doors, junctions 
				
			} // artifacts 
				
				
				
		} // end of map file

		if(badFile)
		{
			return parseWldFile3(fileName);
		}
			
		//go through all the doors, and integrate them with the lines, splitting the lines
		//if necessary
		doors.resetIterator();
		
		while(doors.readNext(line1))
		{
			xd1line = line1.pt1.x;
			yd1line = line1.pt1.y;
			xd2line = line1.pt2.x;
			yd2line = line1.pt2.y;
			
			lines.resetIterator();
			
			while(lines.readNext(line2))
			{
				x1line = line2.pt1.x;
				y1line = line2.pt1.y;
				x2line = line2.pt2.x;
				y2line = line2.pt2.y;
				
				temp1 = (y1line - y2line) * xd1line - (x1line  - x2line) * yd1line 
					+ (x1line - x2line) * y1line - (y1line - y2line) * x1line;
				
				
				temp2 = (y2line - y1line) * xd2line - (x2line  - x1line) * yd2line 
					+ (x2line - x1line) * y2line - (y2line - y1line) * x2line;
				
				if (temp1 == 0 && temp2 == 0) // door is on this wall 
				{
					// break the wall 
					
					dist1 = (xd1line - x1line) *  (xd1line - x1line) + (yd1line - y1line) * (yd1line - y1line);
					dist2 = (xd2line - x1line) *  (xd2line - x1line) + (yd2line - y1line) * (yd2line - y1line);
					// determine the edges of the braked line.
					// the shortest segment from one edge of the wall to the 2 edges of the door is taken 
					
					if (dist1 < dist2)
					{
						line2.setPoints(x1line,y1line,xd1line,yd1line);
						_listLines.push(line2);
						line2.setPoints(xd2line,yd2line,x2line,y2line);
						_listLines.push(line2);
						
						//fprintf(fwld, "%i %i %i %i\n", x1line, y1line, xd1line, yd1line);
						//fprintf(fwld,  "%i %i %i %i\n", xd2line, yd2line, x2line, y2line);
						
					}
					else
					{
						line2.setPoints(x1line,y1line,xd2line,yd2line);
						_listLines.push(line2);
						line2.setPoints(xd1line,yd1line,x2line,y2line);
						_listLines.push(line2);
						
						//fprintf(fwld, "%i %i %i %i\n", x1line, y1line, xd2line, yd2line);
						//fprintf(fwld,  "%i %i %i %i\n", xd1line, yd1line, x2line, y2line);
					}
				}
				else
				{
					//printf("line sans door\n");
					line2.setPoints(x1line,y1line,x2line,y2line);
					_listLines.push(line2);
					//fprintf(fwld,"%i %i %i %i\n", x1line, y1line, x2line, y2line); // line do not contain this door 
				}
				
			}
			
		}
		
		//figure out the width and height of the map
		long minX = 0,maxX = 0, minY = 0,maxY = 0;
		_listLines.resetIterator();
		
		if(_listLines.readHead(line1))
		{
			minX = SosUtil::minVal(line1.pt1.x,line1.pt2.x);
			maxX = SosUtil::maxVal(line1.pt1.x,line1.pt2.x);
			minY = SosUtil::minVal(line1.pt1.y,line1.pt2.y);
			maxY = SosUtil::maxVal(line1.pt1.y,line1.pt2.y)	;	
		}

		while(_listLines.readNext(line1))
		{
			minX = SosUtil::minVal(minX,SosUtil::minVal(line1.pt1.x,line1.pt2.x));
			maxX = SosUtil::maxVal(maxX,SosUtil::maxVal(line1.pt1.x,line1.pt2.x));
			minY = SosUtil::minVal(minY,SosUtil::minVal(line1.pt1.y,line1.pt2.y));
			maxY = SosUtil::maxVal(maxY,SosUtil::maxVal(line1.pt1.y,line1.pt2.y))	;
		}

		_origin.x = minX;
		_origin.y = minY;

		_width = maxX - minX +1;
		_height = maxY - minY +1;
		
		return true;
	}

	//this parses files from the Saphira Mapper program
	bool parseWldFile3(char* fileName)
	{
		LOG<<"In parseWldFile3";
		ifstream in;
		in.open(fileName);
		if(!in.is_open())
			return false;

		const int numEntries = 15;
		char validEntriesStatic[numEntries][20]={"width","height","originpad","start","attachid",
			"end","push","pop","position","change","line","chair","circle","text","goal"};

		char **validEntries = 0;
		int i = 0;//used in for loops

		validEntries = new char*[numEntries];
		for( i = 0; i< numEntries; i++)
		{
			validEntries[i] = validEntriesStatic[i];
		}

		List<int> states;
		states.setModeStack();
		_listPushedPositions.clear();

		PointXYZ pt(0,0,0),pt2(0,0,0);
		PointXYZ changedPt(0,0,0);
		_listPushedPositions.push(pt);

		LineXYLong line, line2;

		char buffer[100] = "";
		bool fileIsBad = false;
		int entryType = -1;

		bool useChangedVar = false;
		bool firstElementIsInBuffer = false;

		int currentState = -1;
		const int STATE_WIDTH = 0, STATE_HEIGHT = 1,STATE_ORIGIN = 2,STATE_START=3,STATE_ATTACHID=4,STATE_END = 5,
			STATE_PUSH = 6, STATE_POP = 7,STATE_POSITION = 8,STATE_CHANGE = 9,STATE_LINE = 10,
			STATE_CHAIR = 11,STATE_CIRCLE = 12, STATE_TEXT = 13,STATE_GOAL = 14;

		in>>buffer;

		while(!fileIsBad && !in.eof() )
		{
			
			entryType = SosUtil::stringInArrayNoCase(buffer,validEntries,numEntries);

			LOG<<"Got token "<<buffer<<" of type"<<entryType;
			//Circle is an exception to the rule, so deal with it separately
			states.readHead(currentState);
			LOG<<"currentState = "<<currentState;
			if(entryType == -1 && currentState == STATE_CIRCLE && SosUtil::is_numeric(buffer))
			{
				LOG<<"set entryType to STATE_LINE";
				entryType = STATE_LINE;
				firstElementIsInBuffer = true;
			}

			switch(entryType)
			{
			case STATE_WIDTH://width
				in>>buffer;
				if(!SosUtil::is_integer(buffer))
				{
					fileIsBad = true;
					break;
				}

				_width = atol(buffer);
				LOG<<"got width "<<_width;

				break;
			case STATE_HEIGHT://height
				in>>buffer;
				if(!SosUtil::is_integer(buffer))
				{
					fileIsBad = true;
					break;
				}

				_height = atol(buffer);
				LOG<<"got height "<<_height;
				break;
			case STATE_ORIGIN://origin
				in>>buffer;
				if(!SosUtil::is_integer(buffer))
				{
					fileIsBad = true;
					break;
				}

				_origin.x = atol(buffer);

				in>>buffer;
				if(!SosUtil::is_integer(buffer))
				{
					fileIsBad = true;
					break;
				}

				_origin.y = atol(buffer);

				LOG<<"Got origin ("<<_origin.x<<","<<_origin.y<<")";
				break;
			case STATE_START://start
				states.push(STATE_START);

				break;
			case STATE_ATTACHID://attachid
				if(!states.readHead(currentState) || currentState != STATE_START)
				{
					fileIsBad = true;
					break;
				}

				//read and ignore the stuff inside the attach id - there's 5 items in there
				for(i = 0; !fileIsBad && i< 5; i++)
				{
					in>>buffer;
					if(!SosUtil::is_integer(buffer))
					{
						fileIsBad = true;						
					}
				}

				LOG<<"Skipped 5 items of attach id";
				if(fileIsBad)
				{
					break;
				}

				break;
			case STATE_END://end
				states.popHead(currentState);

				useChangedVar = false;

				//if the last state was circle, then pop one more time
				if(currentState == STATE_CIRCLE)
					states.popHead(currentState);
				changedPt.x = 0;
				changedPt.y = 0;
				changedPt.value = 0;

				break;
			case STATE_PUSH://push
				in>>buffer;
				if(!SosUtil::is_integer(buffer))
				{
					fileIsBad = true;
					break;
				}

				pt.x = atol(buffer);

				in>>buffer;
				if(!SosUtil::is_numeric(buffer))
				{
					fileIsBad = true;
					break;
				}

				pt.y = atol(buffer);
				in>>buffer;
				if(!SosUtil::is_numeric(buffer))
				{
					fileIsBad = true;
					break;
				}

				//value is the angle
				pt.value = (float)atof(buffer);
				_listPushedPositions.push(pt);
				
				changedPt = pt;

				LOG<<"Pushed position ("<<pt.x<<","<<pt.y<<")";
				
				break;
			case STATE_POP://pop
				_listPushedPositions.popHead(pt);

				LOG<<"Popped position";

				//there should always be at least one point on this list
				//since one was artificially pushed on before reading the file
				if(_listPushedPositions.getListSize() < 1)
				{
					fileIsBad = true;
					break;
				}

				_listPushedPositions.readHead(changedPt);

				break;
			case STATE_POSITION://position
				if(states.getListSize() > 0)
				{
					fileIsBad = true;
					break;
				}

				in>>buffer;
				if(!SosUtil::is_integer(buffer))
				{
					fileIsBad = true;
					break;
				}

				_robotPos.x = atol(buffer);

				in>>buffer;
				if(!SosUtil::is_integer(buffer))
				{
					fileIsBad = true;
					break;
				}

				_robotPos.y = atol(buffer);

				in>>buffer;//the angle of the robot 
				if(!SosUtil::is_integer(buffer))
				{
					fileIsBad = true;
					break;
				}

				_robotPos.value = (float)atol(buffer);
				LOG<<"Got robot position ("<<_robotPos.x<<","<<_robotPos.y<<")";
				
				break;
			case STATE_CHANGE://change - TO DO
							
				in>>buffer;
				if(!SosUtil::is_integer(buffer))
				{
					fileIsBad = true;
					break;
				}
				LOG<<"read "<<buffer;

				useChangedVar = true;

				pt.x = atol(buffer);

				in>>buffer;
				if(!SosUtil::is_integer(buffer))
				{
					fileIsBad = true;
					break;
				}
				LOG<<"read "<<buffer;

				pt.y = atol(buffer);

				in>>buffer;
				if(!SosUtil::is_numeric(buffer))
				{
					fileIsBad = true;
					break;
				}

				LOG<<"read "<<buffer;
				pt.value = (float)atof(buffer);

				//now rotate the point by the current changed angle
				/*
				 Rotation:
				 x = xCos(A) + ySin(A)
				 y = yCos(A) - xSin(A)
				 */

				if(changedPt.value != 0)
				{	
					pt2.x = (long)(pt.x * SosUtil::cos(changedPt.value*-1) + pt.y * SosUtil::sin(changedPt.value*-1));
					pt2.y = (long)(pt.y * SosUtil::cos(changedPt.value*-1) + pt.x * SosUtil::sin(changedPt.value*-1));
				}
				else
				{
					pt2 = pt;
				}
			
				changedPt.x = changedPt.x + pt2.x;
				changedPt.y = changedPt.y + pt2.y;
				changedPt.value = changedPt.value + pt.value;

				LOG<<"set changedPt to ("<<changedPt.x<<","<<changedPt.y<<") "<<changedPt.value;

				break;
			case STATE_LINE://line
				if(!states.readHead(currentState) || 
					(currentState != STATE_START && currentState != STATE_CIRCLE))
				{
					fileIsBad = true;
					break;
				}

				if(!useChangedVar)
					_listPushedPositions.readHead(pt);
				else
					pt = changedPt;

				if(!firstElementIsInBuffer)
				{
					in>>buffer;
					if(!SosUtil::is_integer(buffer))
					{
						fileIsBad = true;
						break;
					}
				}
				else
				{
					firstElementIsInBuffer = false;
				}

				line.pt1.x = atol(buffer);

				in>>buffer;
				if(!SosUtil::is_integer(buffer))
				{
					fileIsBad = true;
					break;
				}

				line.pt1.y = atol(buffer);

				in>>buffer;
				if(!SosUtil::is_integer(buffer))
				{
					fileIsBad = true;
					break;
				}

				line.pt2.x = atol(buffer);

				in>>buffer;
				if(!SosUtil::is_integer(buffer))
				{
					fileIsBad = true;
					break;
				}

				line.pt2.y = atol(buffer);

				//now rotate the line
				/*
				 Rotation:
				 x = xCos(A) + ySin(A)
				 y = yCos(A) - xSin(A)
				 */

				LOG<<"Line before rotation ("<<line.pt1.x<<","<<line.pt1.y<<") -> ("<<line.pt2.x<<","<<line.pt2.y<<")";

				if(pt.value != 0)
				{	
					//translate pt1 to (0,0)
					line2.pt1.x = 0;
					line2.pt1.y = 0;
					line2.pt2.x = line.pt2.x - line.pt1.x;
					line2.pt2.y = line.pt2.y - line.pt1.y;

					//rotate point 2 of the line around (0,0)
					
					pt2.x = (long)(line2.pt2.x  * SosUtil::cos(pt.value*-1) + line2.pt2.y * SosUtil::sin(pt.value*-1));
					pt2.y = (long)(line2.pt2.y * SosUtil::cos(pt.value*-1) + line2.pt2.x * SosUtil::sin(pt.value*-1));

					//translate the point back from (0,0)
					pt2.x += line.pt1.x;
					pt2.y += line.pt1.y;


					line.pt2.x = pt2.x;
					line.pt2.y = pt2.y;
				}

				LOG<<"Line after rotation ("<<line.pt1.x<<","<<line.pt1.y<<") -> ("<<line.pt2.x<<","<<line.pt2.y<<")";

				//now translate the whole line
				line.pt1.x += pt.x;
				line.pt1.y += pt.y;
				line.pt2.x += pt.x;
				line.pt2.y += pt.y;

				LOG<<"Line after translation ("<<line.pt1.x<<","<<line.pt1.y<<") -> ("<<line.pt2.x<<","<<line.pt2.y<<")";


				_listLines.push(line);
				LOG<<"Pushed line ("<<line.pt1.x<<","<<line.pt1.y<<") -> "<<line.pt2.x<<","<<line.pt2.y<<")";

				break;
			case STATE_CHAIR://chair
				if(!states.readHead(currentState) || currentState != STATE_START)
				{
					fileIsBad = true;
					break;
				}

				//read and ignore the first 7 items in the chair
				for(i = 0; !fileIsBad && i< 7; i++)
				{
					in>>buffer;
					if(!SosUtil::is_numeric(buffer))
					{
						fileIsBad = true;						
					}
				}

				if(fileIsBad)
					break;

				_listPushedPositions.readHead(pt);

				//read the 4 lines of the chair
				for(i = 0; i< 4; i++)
				{
					in>>buffer;
					if(!SosUtil::is_integer(buffer))
					{
						fileIsBad = true;						
					}

					line.pt1.x = atol(buffer) + pt.x;

					in>>buffer;
					if(!SosUtil::is_integer(buffer))
					{
						fileIsBad = true;						
					}

					line.pt1.y = atol(buffer) + pt.y;

					in>>buffer;
					if(!SosUtil::is_integer(buffer))
					{
						fileIsBad = true;						
					}

					line.pt2.x = atol(buffer) + pt.x;

					in>>buffer;
					if(!SosUtil::is_integer(buffer))
					{
						fileIsBad = true;						
					}

					line.pt2.y = atol(buffer) + pt.y;

					_listLines.push(line);
					LOG<<"Pushed chair line ("<<line.pt1.x<<","<<line.pt1.y<<") -> "<<line.pt2.x<<","<<line.pt2.y<<")";
				}


				break;
			case STATE_CIRCLE://circle
			//	fileIsBad = true;//take out
				//	break;

				if(!states.readHead(currentState) || currentState != STATE_START)
				{
					fileIsBad = true;
					break;
				}
				//read and ignore the first 3 items in the text - we don't use it
				for(i = 0; !fileIsBad && i< 3; i++)
				{
					in>>buffer;
					LOG<<"Read circle item "<<i<<": "<<buffer;
					if(!SosUtil::is_numeric(buffer))
					{
						fileIsBad = true;						
					}
				}

				states.push(STATE_CIRCLE);
				

				break;
			case STATE_TEXT://text
				if(!states.readHead(currentState) || currentState != STATE_START)
				{
					fileIsBad = true;
					break;
				}

				//read and ignore the first 3 items in the text - we don't use it
				for(i = 0; !fileIsBad && i< 3; i++)
				{
					in>>buffer;
					LOG<<"Read text item "<<i<<": "<<buffer;
					if(i < 2 && !SosUtil::is_integer(buffer))
					{
						fileIsBad = true;						
					}
				}

				LOG<<"Ignored 3 items of text";

				break;
			case STATE_GOAL://goal
				if(!states.readHead(currentState) || currentState != STATE_START)
				{

					fileIsBad = true;
					break;
				}

				//read and ignore the first 3 items in the goal - we don't use it
				for(i = 0; !fileIsBad && i< 3; i++)
				{
					in>>buffer;
					LOG<<"Read goal item "<<i<<": "<<buffer;
					if(i < 2 && !SosUtil::is_integer(buffer))
					{
						fileIsBad = true;						
					}
				}

				LOG<<"Ignored 3 items of goal";
				break;
			default:
				LOG<<"Error! entryType = "<<entryType<<" for token "<<buffer;
				fileIsBad = true;
			}
			in>>buffer;
		}

		if(fileIsBad)
		{
			LOG<<"Failed on token "<<buffer;
			in.close();
			_listLines.clear();
			_listPushedPositions.clear();
			return false;
		}

		return true;
	}
	
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
	
	bool is_numeric(char * str) const
	{
		if ( strspn ( str, "0123456789.+-e" ) != strlen(str) ) 
			return ( false) ;
		else
			return ( true) ;
	}
	
	List<LineXYLong> 	_listLines ;
	List<PointXYZ> 	_listPushedPositions ;
	
	long		_width, _height;
	PointXYLong _origin;
	PointXYZ	_robotPos;
	bool		_hasRobotPos;

	DEF_LOG
	
	
};

#endif