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

#ifndef RobotRunFileHelperStage_H
#define RobotRunFileHelperStage_H


#include <List>
#include <math.h>
#include <fstream.h>
#include "SosUtil.h"
#include "commonDefs.h"
#include "RobotRunFileHelper.h"
#include "../logger/Logger.h"

//RobotRunFileHelper both reads and writes Robot Run files for the Map Viewer application, http://mapviewer.skynet.ie
//A RobotRun file contains the path that a robot took as a list of Poses, and optionally the sonar and/or laser
//range data that it read at each pose.  See the bottom of this file for an example of the format, or see the above 
//website for more information
class RobotRunFileHelperStage: public RobotRunFileHelper
{
public:
	//constructor
	RobotRunFileHelperStage()
	{
		GET_FILE_LOG
		reset();

		_parseSonar = _parseLaser = _parsePosition = true;		
	}

	//destructor
	~RobotRunFileHelperStage()
	{
		reset();
	}

	void setParseSonar(bool val)
	{
		_parseSonar = val;
	}

	
	void setParseLaser(bool val)
	{
		_parseLaser = val;
	}

	
	void setParsePosition(bool val)
	{
		_parsePosition = val;
	}
	//parses the file at the path "fileName"
	//Returns true if the file was semantically correct, i.e. if it was parsed correctly
	//Returns false if the file cannot be opened or if it is not correctly written
	bool parseFile(char* fileName)
	{
		reset();

		char errorBuf[200] = {0};
		
		int i = 0, x = 0;//used in for loops
		for(i = 0; i< 1024; i++)
		{
			_errorToken[i] = '\0';
		}

		LOG<<"Reset Error Token and it is now: "<<_errorToken;
		

		if(fileName == 0)
		{
			strcpy(_errorToken,"Could not open file");
			return false;
		}

		ifstream in;
		in.open(fileName);
		if(!in.is_open())
		{
			strcpy(_errorToken,"Could not open file");
			return false;
		}

		const int numEntries = 3;
		char validEntriesStatic[numEntries][20]={"position","sonar","laser"};

		char **validEntries = 0;
		
		validEntries = new char*[numEntries];
		for( i = 0; i< numEntries; i++)
		{
			validEntries[i] = validEntriesStatic[i];
		}

		const int TOKEN_POSITION = 0, TOKEN_SONAR = 1, TOKEN_LASER = 2;

		const int BASE_STATE = -1, STATE_SONAR = 1, STATE_LASER = 2,STATE_POSITION = 0;
		
		int currentState = BASE_STATE;
//		char buffer[2048];
		char line[2048] = {0};
		char token[1024] = {0};
		bool fileIsBad = false;
		int entryType = -1;
		int count = 0;

		int numSonars = -1, numLasers = -1, numSensorsRead = 0, numSensors = 0;

		std::list<SensorReading> * sensorList = 0;

		SosPose pose(0,0,0);
		SosPose lastPose(0,0,0);
		double * data = 0;
		SensorReading* sonarReading = 0;
		SensorReading poseReading(0);

		bool readSonarLayout = false, readLaserLayout = false;

		long tokenPos = -1 , sonarTokenPos = -1;

		long tokenPos2 = -1;

		const char delimiters[] = " \t";

		long lineCounter = 0;

		

		const int RECORD_TYPE_POS = 3;//sonar, laser or position should be in position 4
		const int NUM_SONARS_POS = 6;
		const int NUM_INITIAL_TOKENS = 7;
		const int NUM_POSITION_TOKENS = 9;

		char** initialTokens = 0;
		initialTokens = new char*[NUM_INITIAL_TOKENS];
		for(i = 0; i< NUM_INITIAL_TOKENS; i++)
		{
			initialTokens[i] = new char[100];
		}

		char** sonarReadingTokens = 0;
		char** positionReadingTokens = 0;
		long*  sonarRanges = 0;

		
		positionReadingTokens = new char*[NUM_POSITION_TOKENS];
		for(i = 0; i<NUM_POSITION_TOKENS; i++)
		{
			positionReadingTokens[i] = new char[100];	
		}

		char** sonarPoses = 0;
		char** laserPoses = 0;

		in.getline(line, 2048);

		int p = 0;

		

		while(!in.eof() && !fileIsBad)
		{		
			lineCounter ++;
			LOG<<"Line "<<lineCounter<<": "<<line;

		//	SosUtil::trim(buffer,line);
		//	strcpy(line,buffer);

			currentState = BASE_STATE;

			if(line[0] == '#')//skip comments
			{
				in.getline(line, 2048);
				continue;
			}

			tokenPos = SosUtil::tokenise(line,initialTokens,delimiters,2,1,NUM_INITIAL_TOKENS);


			//LOG<<"Here are the "<<NUM_INITIAL_TOKENS<<" tokens";
			//for(int p = 0; p < NUM_INITIAL_TOKENS;p++)
			//{
			//	LOG<<'['<<p<<']'<<initialTokens[p];
			//}


			if(tokenPos == -1)
			{
				//don't recognise this line
				in.getline(line, 2048);
				continue;
			}

			if(stricmp(initialTokens[RECORD_TYPE_POS],"position") ==0)
			{
				if(true)//_parsePosition)
				{
					LOG<<"set currentState to "<<STATE_POSITION;
					currentState = STATE_POSITION;
				}
			}
			else if(stricmp(initialTokens[RECORD_TYPE_POS],"sonar") ==0)
			{
				if(_parseSonar)
						currentState = STATE_SONAR;
			}
			else if(stricmp(initialTokens[RECORD_TYPE_POS],"laser") ==0)
			{
				if(_parseLaser)
					currentState = STATE_LASER;
			}
			
			switch(currentState)
			{
			case BASE_STATE:
				LOG<<"Don't recognise the line: "<<line;
				in.getline(line, 2048);
				continue;

				break;
			case STATE_SONAR:
				//we know that the word "sonar" is at the position 'tokenPos' in the String 'line'
				//So we have to skip the "sonar" token, skip the next token which is some index
				//that we don't need, then skip the third token, which is a timestamp

				//If we haven't gotten the position of the sonars yet then get them
				if(numSonars == -1)
				{
					if(!SosUtil::is_integer(initialTokens[NUM_SONARS_POS]))
					{
						fileIsBad = true;
						strcpy(_errorToken,"Error in sonar record on line ");
						strcat(_errorToken,ltoa(lineCounter,errorBuf,10));
						strcat(_errorToken,". The token '");
						strcat(_errorToken,token);
						strcat(_errorToken,"' should be an integer");
						break;
					}

					numSonars = atol(initialTokens[NUM_SONARS_POS]);

					sonarPoses = new char*[numSonars * 3];
					for(x = 0; x < numSonars * 3; x++)
					{
						sonarPoses[x] = new char[20];
					}

					sonarTokenPos = SosUtil::tokenise(line + tokenPos, sonarPoses, delimiters,2,1,numSonars * 3);

			//		LOG<<"Here are the "<<numSonars<<" tokens";
			//		for(int p = 0; p < numSonars - 2;p+=3)
			//		{
			//			LOG<<'['<<p<<"] ("<<sonarPoses[p]<<","<<sonarPoses[p+1]<<") "<<sonarPoses[p+2];
			//		}

					if(sonarTokenPos == -1)
					{
						fileIsBad = true;
						strcpy(_errorToken,"Error in sonar record on line ");
						strcat(_errorToken,ltoa(lineCounter,errorBuf,10));
						strcat(_errorToken,". There are not enough sonar poses");
						break;
					}

					sonarTokenPos += tokenPos;

					LOG<<"sonarTokenPos = "<<sonarTokenPos<<" and tokenPos = "<<tokenPos;
					for(x = 0; x < numSonars * 3; x++)
					{
						if(!SosUtil::is_numeric(sonarPoses[x]))
						{
							fileIsBad = true;
							break;
						}
					}
					if(fileIsBad)
					{
						strcpy(_errorToken,"Error in sonar record on line ");
						strcat(_errorToken,ltoa(lineCounter,errorBuf,10));
						strcat(_errorToken,".Sonar pose must be numeric");
						break;
					}

					for(x = 0; x < numSonars * 3; x+=3)
					{
						pose.setX(atof(sonarPoses[x]) * 1000);
						pose.setY(atof(sonarPoses[x+1]) * 1000);

						//angle readings in the Player/Stage log file are in radians
						//but we need them in degrees
						pose.setTh(SosUtil::radToDeg(atof(sonarPoses[x+2])));

						LOG<<"Pushed sonar pose ("<<pose.getX()<<","<<pose.getY()<<") ["<<pose.getTh()<<"]";
						cout<<"\n["<<x<<"] Pushed sonar pose ("<<pose.getX()<<","<<pose.getY()<<") ["<<pose.getTh()<<"]";
						_sonarPosesList.push_back(pose);
					}

					sonarReadingTokens = new char*[numSonars];
					for(x = 0; x< numSonars; x++)
					{
						sonarReadingTokens [x] = new char[20];
					}

					sonarReading = new SensorReading(numSonars);
					sonarRanges = new long[numSonars];
				}

				LOG<<"tokenising range line "<<line + sonarTokenPos<<endl;

				for(x = 0; x < numSonars; x++)
				{
					sonarReadingTokens[x][0] = '\0';
				}
				tokenPos2 = SosUtil::tokenise(line + sonarTokenPos,sonarReadingTokens,delimiters,2,2,numSonars+1);
				
				if(tokenPos2 == -1)
				{
					fileIsBad = true;
					strcpy(_errorToken,"Error in sonar record on line ");
					strcat(_errorToken,ltoa(lineCounter,errorBuf,10));
					strcat(_errorToken,". There are not enough sonar pose readings");
					break;
				}
				tokenPos2 += sonarTokenPos;
				

				for(x = 0; x < numSonars; x++)
				{
					if(!SosUtil::is_numeric(sonarReadingTokens[x]))
					{
						fileIsBad = true;
						break;
					}
					sonarRanges[x] = (long)( atof(sonarReadingTokens[x]) *1000 );
					LOG<<"Range ["<<x<<"] = "<<sonarReadingTokens[x]<<" metres = "<<sonarRanges[x]<<" mm";
					
				}

				
				if(fileIsBad)
				{
					strcpy(_errorToken,"Error in sonar record on line ");
					strcat(_errorToken,ltoa(lineCounter,errorBuf,10));
					strcat(_errorToken,". Sonar range reading must be numeric.");
					break;
				}

				sonarReading->setPose(lastPose);

				sonarReading->setRange(sonarRanges);
			
				_sonarReadingsList.push_back(*sonarReading);

				LOG<<"Pushed sonar reading, list size is now "<<_sonarReadingsList.size();
								
				break;
			case STATE_LASER:
				//TO DO
				break;
			case STATE_POSITION:
				LOG<<"In position";
				tokenPos = SosUtil::tokenise(line,positionReadingTokens,delimiters,2,7,NUM_POSITION_TOKENS);

				if(tokenPos == -1)
				{
					fileIsBad = true;
					strcpy(_errorToken,"Error in position record on line ");
					strcat(_errorToken,ltoa(lineCounter,errorBuf,10));
					strcat(_errorToken,". There are not enough position readings");
					break;					
				}
				fileIsBad = false;

				for(i = 0; i< 3; i++)
				{
					if(!SosUtil::is_numeric(positionReadingTokens[i]))
					{
						LOG<<"The token "<<positionReadingTokens[i]<<" is not numeric";
						fileIsBad = true;
						break;
					}
				}

				if(fileIsBad)
				{
					strcpy(_errorToken,"Error in position record on line ");
					strcat(_errorToken,ltoa(lineCounter,errorBuf,10));
					strcat(_errorToken,". Position reading must be numeric");
					break;	
				}
				LOG<<"Set the pose("<<positionReadingTokens[0]<<","<<positionReadingTokens[1]<<")["<<positionReadingTokens[2]<<"]";
				lastPose.setPose(atof(positionReadingTokens[0])*1000,
									atof(positionReadingTokens[1])*1000,
									SosUtil::radToDeg(atof(positionReadingTokens[2])));

				if(_parsePosition)
				{
					poseReading.setPose(lastPose);
				//	poseReading.setRange(0);
					_posesList.push_back(poseReading);
				}



				break;
			
			default:
				
				break;

			}

			if(fileIsBad)
			{
				LOG<<"Error: file parse failed on: "<<line<<", and error token is: "<<_errorToken;
			}

			in.getline(line, 2048);
		}

		
		for(i = 0; i< NUM_INITIAL_TOKENS; i++)
		{
			delete []initialTokens[i];
		}
		delete[] initialTokens;

		if(sonarPoses != 0)
		{
			for(x = 0; x < numSonars; x++)
			{
				delete []sonarPoses[x];
			}
			delete []sonarPoses;
			sonarPoses= 0;
		}

		if(sonarReadingTokens != 0)
		{
			for(x = 0; x< numSonars; x++)
			{
				delete[] sonarReadingTokens [x];
			}
			delete[] sonarReadingTokens;
			sonarReadingTokens = 0;
		}

		if(positionReadingTokens != 0)
		{
			for(i = 0; i<NUM_POSITION_TOKENS; i++)
			{
				delete [] positionReadingTokens[i];	
			}
			delete []positionReadingTokens;
		}

		if(fileIsBad)
		{
			LOG<<"At end of parse method, file is bad, buffer = "<<line<<", and error token = "<<_errorToken;
			reset();
			in.close();

			return false;
		}

		if(sonarReading != 0)
			delete sonarReading;

		if(sonarRanges != 0)
			delete [] sonarRanges;
		
		in.close();
		LOG<<"File parsed correctly";
		return true;
	}



//--------------------------------------------------

	//write the information stored in the lists and robot radius to a the file "fileName".
	//For sonar data to be written, there must be information both on the positions of the sonars 
	//using the pushSonarPose method, and on the range data from the sonars using the pushSonarReading method.
	//The same goes for laser data.
	//For just Pose data, only Pose data has to be entered, using the pushPoseReading method
	bool writeToFile(char* fileName)
	{
		return false;
	}

private:
	bool							_parseSonar;
	bool							_parseLaser;
	bool							_parsePosition;

	DEF_LOG
};


#endif