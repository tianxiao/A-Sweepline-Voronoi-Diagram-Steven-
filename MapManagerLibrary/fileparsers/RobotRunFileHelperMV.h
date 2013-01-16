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

#ifndef RobotRunFileHelperMV_H
#define RobotRunFileHelperMV_H

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
class RobotRunFileHelperMV : public RobotRunFileHelper
{
public:
	//constructor
	RobotRunFileHelperMV()
	{
		GET_FILE_LOG
		reset();
	}

	//destructor
	~RobotRunFileHelperMV()
	{
		reset();
	}

	//parses the file at the path "fileName"
	//Returns true if the file was semantically correct, i.e. if it was parsed correctly
	//Returns false if the file cannot be opened or if it is not correctly written
	bool parseFile(char* fileName)
	{
		reset();
		
		int i = 0;//used in for loops
		for(i = 0; i< 1024; i++)
		{
			_errorToken[i] = '\0';
		}

		LOG<<"Reset Error Token and it is now: "<<_errorToken;
		

		if(fileName == 0)
		{
			LOG<<"File name is null, returning false";
			strcpy(_errorToken,"Could not open file");
			return false;
		}

		ifstream in;
		in.open(fileName);
		if(!in.is_open())
		{
			LOG<<"File couldn't be opened, returning false";
			strcpy(_errorToken,"Could not open file");
			return false;
		}

		LOG<<"file is open, going to parse it"<<endl;
		const int numEntries = 10;
		char validEntriesStatic[numEntries][20]={"numsonars","data","start","end","numlasers","robotradius",
												"sonar","laser"};

		char **validEntries = 0;
		
		validEntries = new char*[numEntries];
		for( i = 0; i< numEntries; i++)
		{
			validEntries[i] = validEntriesStatic[i];
		}

		const int TOKEN_NUMSONAR = 0, TOKEN_DATA = 1, TOKEN_START = 2,TOKEN_END = 3,
			TOKEN_NUMLASER = 4, TOKEN_ROBOTRADIUS = 5, TOKEN_SONAR = 6, TOKEN_LASER = 7;

		const int STATE_SONARLIST = 0, STATE_LASERLIST = 2,STATE_DATA = 3;

		const int BASE_STATE = -1;
		int currentState = BASE_STATE;
		char buffer[250];
		bool fileIsBad = false;
		int entryType = -1;

		in>>buffer;

		if(stricmp(buffer,"robotrun") != 0)
		{
			fileIsBad = true;
			strcpy(_errorToken,buffer);
			LOG<<"Error: file parse failed on: "<<buffer<<", and error token is: "<<_errorToken;
		}

		int numSonars = -1, numLasers = -1, numSensorsRead = 0, numSensors = 0;

		std::list<SensorReading> * sensorList = 0;

		SosPose pose(0,0,0);
		double * data = 0;
		SensorReading* sensorReading = 0;

		bool readSonarLayout = false, readLaserLayout = false;

		LOG<<"FileIsBad = "<<fileIsBad<<endl;
		in>>buffer;

		while(!in.eof() && !fileIsBad)
		{		
			LOG<<"currentState = "<<currentState<<endl;
			switch(currentState)
			{
			case BASE_STATE:
				entryType = stringInArrayNoCase(buffer,validEntries,numEntries);
				LOG<<"entryType = "<<entryType<<endl;

				switch(entryType)
				{
				case TOKEN_NUMSONAR:
					in>>buffer;
					if(!is_integer(buffer))
					{
						fileIsBad = true;
						strcpy(_errorToken,buffer);
						break;
					}
					numSonars = atol(buffer);
					break;
				case TOKEN_DATA :
					in>>buffer;
					if(stricmp(buffer,"sonar") == 0 && numSonars > 0 && readSonarLayout)
					{
						//LOG<<"Made the sensorList point to the sonar readings list";
						sensorList = &_sonarReadingsList;
						numSensors = numSonars;
					}
					else if(stricmp(buffer,"laser") == 0 && numLasers > 0 && readLaserLayout)
					{
						//LOG<<"Made the sensorList point to the laser readings list";
						sensorList = &_laserReadingsList;
						numSensors = numLasers;
					}
					else if(stricmp(buffer,"pose") == 0)
					{				
						//LOG<<"Made the sensorList point to the pose readings list";
						sensorList = &_posesList;
						numSensors = 0;						
					}
					else
					{
						fileIsBad = true;	
						strcpy(_errorToken,buffer);
						break;
					}

					currentState = STATE_DATA;

					LOG<<"Created new sensor reading with size = "<<numSensors;
					sensorReading = new SensorReading(numSensors);
					
					LOG<<"Created the data Array, with size = "<<numSensors + 3;
					data = new double[numSensors + 3];
					break;
				case TOKEN_START:
					in>>buffer;
					if(stricmp(buffer,"sonar") == 0 && numSonars != 0)
					{
						currentState = STATE_SONARLIST;
						_sonarPosesList.clear();
						numSensorsRead = 0;
						numSensors = numSonars;
					}
					else if(stricmp(buffer,"laser") == 0 && numLasers != 0)
					{
						currentState = STATE_LASERLIST;
						_laserPosesList.clear();
						numSensorsRead = 0;
						numSensors = numLasers;
					}
					else
					{
						fileIsBad = true;	
						strcpy(_errorToken,buffer);
					}
					break;
				case TOKEN_NUMLASER :
					in>>buffer;
					if(!is_integer(buffer))
					{
						fileIsBad = true;
						break;
					}
					numLasers = atol(buffer);
					break;
				case TOKEN_ROBOTRADIUS :
					in>>buffer;
					if(!is_integer(buffer))
					{
						fileIsBad = true;
						strcpy(_errorToken,buffer);
						break;
					}
					_robotRadius = atol(buffer);
					break;
				default:
					fileIsBad = true;
					break;
				}
				break;
			case STATE_SONARLIST:
				if(is_numeric(buffer) && numSensorsRead <= numSensors)
				{
					pose.setX(atol(buffer));

					in>>buffer;
					if(!is_numeric(buffer))
					{
						fileIsBad = true;
						strcpy(_errorToken,buffer);
						break;
					}
					pose.setY(atol(buffer));

					in>>buffer;
					if(!is_numeric(buffer))
					{
						fileIsBad = true;
						strcpy(_errorToken,buffer);
						break;
					}
					pose.setTh(atol(buffer));

					_sonarPosesList.push_back(pose);
					numSensorsRead++;
				}
				else if(stricmp(buffer, "end") == 0)
				{
					currentState = BASE_STATE;
					numSensorsRead = numSensors = 0;
					readSonarLayout = true;
				}
				else
				{
					fileIsBad = true;
					strcpy(_errorToken,buffer);
				}

				break;
			case STATE_LASERLIST:
				if(is_numeric(buffer) && numSensorsRead <= numSensors)
				{
					pose.setX(atol(buffer));

					in>>buffer;
					if(!is_numeric(buffer))
					{
						fileIsBad = true;
						strcpy(_errorToken,buffer);
						break;
					}
					pose.setY(atol(buffer));

					in>>buffer;
					if(!is_numeric(buffer))
					{
						fileIsBad = true;
						strcpy(_errorToken,buffer);
						break;
					}
					pose.setTh(atol(buffer));

					_laserPosesList.push_back(pose);
					numSensorsRead++;
				}
				else if(stricmp(buffer, "end") == 0)
				{
					currentState = BASE_STATE;
					numSensorsRead = numSensors = 0;
					readLaserLayout = true;
				}
				else
				{
					fileIsBad = true;
					strcpy(_errorToken,buffer);
				}

				break;
			case STATE_DATA:
				if(sensorList == 0 || data == 0 ||sensorReading == 0)
				{
					LOG<<"Setting fileIsBad to true, sensorList = "<<sensorList<<", data = "<<data<<", sensorReading = "<<sensorReading;
					//this would be more of a parser error-cannot get here without the file being correct
					fileIsBad = true;
					strcpy(_errorToken,buffer);
					break;
				}
				LOG<<"buffer = "<<buffer<<", numSensors = "<<numSensors<<endl;
				if(is_numeric(buffer))
				{
					data[0] = atof(buffer);

					for(i = 1; i < numSensors+3; i++)
					{
						in>>buffer;
						if(!is_numeric(buffer))
						{
							fileIsBad = true;
							strcpy(_errorToken,buffer);
							LOG<<"Found a bad token1: "<<buffer<<endl;
							break;
						}
						data[i] = atof(buffer);
					}

					LOG<<"After for loop"<<endl;

					sensorReading->setPose(data[0],data[1],data[2]);
					for(i = 0; i<numSensors; i++)
					{
						sensorReading->setRange(i,(long)data[i+3]);
					}

					LOG<<"Before pushing the sensor reading onto the list "<<sensorList<<" when the sonar reading list's address is "<<&_sonarReadingsList<<endl;
					sensorList->push_back(*sensorReading);
					LOG<<"After pushing the sensor reading onto the list"<<endl;
				}
				else if(stricmp(buffer,"end") == 0)
				{
					LOG<<"Found the END token"<<endl;
					currentState = BASE_STATE;
					
					sensorList = 0;
					delete []data;
					data = 0;
					delete sensorReading;
					sensorReading = 0;
				}
				else
				{
					LOG<<"Found a bad token2: "<<buffer;
					fileIsBad = true;
					strcpy(_errorToken,buffer);
					break;
				}
				break;
			default:
				fileIsBad = true;
				strcpy(_errorToken,buffer);
				LOG<<"Found a bad token3: "<<buffer;
				break;

			}

			if(fileIsBad)
			{
				LOG<<"Error: file parse failed on: "<<buffer<<", and error token is: "<<_errorToken;
			}

			in>>buffer;
		}

		if(fileIsBad)
		{
			LOG<<"At end of parse method, file is bad, buffer = "<<buffer<<", and error token = "<<_errorToken;
			reset();
			in.close();

			return false;
		}

		in.close();
		LOG<<"File parsed correctly";
		return true;
	}

	//write the information stored in the lists and robot radius to a the file "fileName".
	//For sonar data to be written, there must be information both on the positions of the sonars 
	//using the pushSonarPose method, and on the range data from the sonars using the pushSonarReading method.
	//The same goes for laser data.
	//For just Pose data, only Pose data has to be entered, using the pushPoseReading method
	bool writeToFile(char* fileName)
	{
		//if an invalid file name is passed in, return false
		if(fileName == 0)
			return false;

		ofstream out;
		out.open(fileName);

		//if we can't open the file, return false
		if(!out.is_open())
			return false;

		out<<"robotrun\n";
		SosPose pose;
		SensorReading reading;
		int numSensors = -1;

		out<<"robotradius "<<_robotRadius<<endl;

		//if there is sonar information on both the layout of the sonars and
		//data collected from the sonars, then write them to the file
		if(_sonarPosesList.size() > 0 && _sonarReadingsList.size() > 0)
		{
			resetSonarPoseIterator();
			numSensors = _sonarPosesList.size();
			out<<"numsonars "<<numSensors;
			out<<"\nstart sonar";

			while(getNextSonarPose(pose))
			{
				out<<"\n"<<pose.getX()<<' '<<pose.getY()<<' '<<pose.getTh();
			}
			out<<"\nend";

			out<<"\nDATA SONAR";
			resetSonarReadingIterator();
			while(getNextSonarReading(reading))
			{
				pose = reading.getPose();

				//output the pose of the robot when the readings were taken
				out<<"\n"<<pose.getX()<<' '<<pose.getY()<<' '<<pose.getTh();
				
				//output the range reading of the sonars
				for(int i = 0; i< numSensors; i++)
				{
					out<<' '<<reading.getRange(i);
				}
			}
			out<<"\nEND";
		}

		//if there is laser information on both the layout of the lasers and
		//data collected from the lasers, then write them to the file
		if(_laserPosesList.size() > 0 && _laserReadingsList.size() > 0)
		{
			resetLaserPoseIterator();
			numSensors = _laserPosesList.size();
			out<<"numlasers "<<numSensors;
			out<<"\nstart laser";

			while(getNextLaserPose(pose))
			{
				out<<"\n"<<pose.getX()<<' '<<pose.getY()<<' '<<pose.getTh();
			}
			out<<"\nend";

			out<<"\nDATA LASER";
			resetSonarReadingIterator();
			while(getNextLaserReading(reading))
			{
				pose = reading.getPose();

				//output the pose of the robot when the readings were taken
				out<<"\n"<<pose.getX()<<' '<<pose.getY()<<' '<<pose.getTh();
				
				//output the range reading of the sonars
				for(int i = 0; i< numSensors; i++)
				{
					out<<' '<<reading.getRange(i);
				}
			}
			out<<"\nEND";
		}

		if(_posesList.size() > 0)
		{
			resetPosesIterator();

			out<<"\nDATA POSE";

			while(getNextPose(reading))
			{
				pose = reading.getPose();
				out<<endl<<pose.getX()<<' '<<pose.getY()<<' '<<pose.getTh();
			}
			out<<"\nEND";

		}


		return true;
	}


private:
	DEF_LOG
};

/*Here is an example file with a few readings
robotrun

numsonars 7
start sonar
100 100 90
120 80 30
130 40 15
130 0 0
130 -40 -15
120 -80 -30
100 -100 -90
end

numlasers 7
start laser
100 100 90
120 80 30
130 40 15
130 0 0
130 -40 -15
120 -80 -30
100 -100 -90
end
robotradius 220
DATA SONAR
0 0 0 870 2349 2500 2500 2500 2500 926 
1 1 0 870 2349 2500 2500 2500 2500 926 
2 3 0 905 2349 2500 2500 2400 2500 894 
4 5 0 905 2500 2500 2500 2400 2500 894 
5 6 0 905 2500 2500 2500 2400 2500 894 
0 0 0 905 2500 2500 2500 2200 2500 895 
END
DATA LASER
0 0 0 870 2349 2500 2500 2500 2500 926 
1 1 0 870 2349 2500 2500 2500 2500 926 
2 3 0 905 2349 2500 2500 2400 2500 894 
4 5 0 905 2500 2500 2500 2400 2500 894 
5 6 0 905 2500 2500 2500 2400 2500 894 
0 0 0 905 2500 2500 2500 2200 2500 895 
END
DATA POSE
0 0 0 
1 1 0 
2 3 0 
4 5 0 
5 6 0 
0 0 0 
END

	DATA SONAR
	0 0 0 870 2349 2500 2500 2500 2500 926 
	1 1 0 870 2349 2500 2500 2500 2500 926 
	END

  */





#endif