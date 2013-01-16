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

#ifndef RobotRunFileHelper_H
#define RobotRunFileHelper_H

#include <List>
#include <math.h>
#include <fstream.h>
#include "SosUtil.h"
#include "commonDefs.h"
#include "../logger/Logger.h"

//RobotRunFileHelper both reads and writes Robot Run files for the Map Viewer application, http://mapviewer.skynet.ie
//A RobotRun file contains the path that a robot took as a list of Poses, and optionally the sonar and/or laser
//range data that it read at each pose.  See the bottom of this file for an example of the format, or see the above 
//website for more information
class RobotRunFileHelper
{
protected:
	//constructor
	RobotRunFileHelper()
	{
		GET_FILE_LOG
		reset();
	}
public:

	//destructor
	virtual ~RobotRunFileHelper()
	{
		reset();
	}

	//reset the object - clear all lists and set all iterators to point to the start of all their corresponding lists
	void reset()
	{
		LOG<<"In RobotRunFileHelper::reset()";

		_laserReadingsList.clear();
		_sonarReadingsList.clear();
		_posesList.clear();
		_sonarPosesList.clear();
		_laserPosesList.clear();

		_sonarReadingsListIterator = _sonarReadingsList.begin();
		_laserReadingsListIterator = _laserReadingsList.begin();
		_posesListIterator = _posesList.begin();
		_sonarPosesListIterator = _sonarPosesList.begin();
		_laserPosesListIterator = _laserPosesList.begin();

		_robotRadius = 0;
		
	}

	//parses the file at the path "fileName"
	//Returns true if the file was semantically correct, i.e. if it was parsed correctly
	//Returns false if the file cannot be opened or if it is not correctly written
	virtual bool parseFile(char* fileName) = 0;

	char* getErrorToken()
	{
		LOG<<"Returning the error token, which is "<<_errorToken<<endl;
		return _errorToken;
	}

	//return the number of sonars on the robot
	int numSonars()
	{
		return _sonarPosesList.size();
	}

	//return the number of lasers on the robot
	int numLasers()
	{
		return _laserPosesList.size();
	}

	//return the number of sonar range readings in the file
	long numSonarReadings()
	{
		return _sonarReadingsList.size();
	}

	//return the number of laser range readings in the file
	long numLaserReadings()
	{
		return _laserReadingsList.size();
	}

	//return the number of pose readings in the file
	long numPoseReadings()
	{
		return _posesList.size();
	}
	
//-----------------------------------
	//reset the iterator for the Sonar range readings list
	void resetSonarReadingIterator()
	{
		_sonarReadingsListIterator = _sonarReadingsList.begin();
	}
	
	//reset the iterator for the Laser range readings list
	void resetLaserReadingIterator()
	{
		_laserReadingsListIterator = _laserReadingsList.begin();
	}

	//reset the iterator for the Pose readings list
	void resetPosesIterator()
	{
		_posesListIterator = _posesList.begin();
	}

//----------------------------------------

	//reset the iterator for the Sonar Pose list.  This list contains the position of each sonar in relation
	//to the direction the robot is facing, and also it's placement relative to the centre of the robot.
	void resetSonarPoseIterator()
	{	
		_sonarPosesListIterator = _sonarPosesList.begin();
	}

	//reset the iterator for the Laser Pose list.  This list contains the position of each laser in relation
	//to the direction the robot is facing, and also it's placement relative to the centre of the robot.
	void resetLaserPoseIterator()
	{	
		_laserPosesListIterator = _laserPosesList.begin();
	}
//----------------------------------------
	//get the next Sonar Range Reading.  Returns false if it has reached the end of the list 
	//(and therefore a valid reading was NOT returned), otherwise returns true
	bool getNextSonarReading(SensorReading& reading)
	{		
		if(_sonarReadingsListIterator == _sonarReadingsList.end())
		{
			return false;
		}

		reading = *_sonarReadingsListIterator;
		_sonarReadingsListIterator++;
		return true;
	}

	//get the next Sonar Range Reading, and remove it from the list.  Can save memory if you only need to 
	//read the reading and discard it.
	//Returns false if it has reached the end of the list 
	//(and therefore a valid reading was NOT returned), otherwise returns true
	bool popSonarReading(SensorReading& reading)
	{		
		if(_sonarReadingsList.size() > 0)
		{
			reading = _sonarReadingsList.front();

			_sonarReadingsList.pop_front();
			return true;
		}
		else
		{
			return false;
		}
	}
//----------------------------------------------
	//get the next Laser Range Reading.  Returns false if it has reached the end of the list 
	//(and therefore a valid reading was NOT returned), otherwise returns true
	bool getNextLaserReading(SensorReading& reading)
	{
		
		if(_laserReadingsListIterator == _laserReadingsList.end())
		{
			return false;
		}

		reading = *_laserReadingsListIterator;
		_laserReadingsListIterator++;
		return true;
	}

	//get the next Laser Range Reading, and remove it from the list.  Can save memory if you only need to 
	//read the reading and discard it.
	//Returns false if it has reached the end of the list 
	//(and therefore a valid reading was NOT returned), otherwise returns true
	bool popLaserReading(SensorReading& reading)
	{
		
		if(_laserReadingsList.size() > 0)
		{
			reading = _laserReadingsList.front();
			_laserReadingsList.pop_front();
			return true;
		}
		else
		{
			return false;
		}
	}

	//----------------------------------------
	
	//returns the pose of a sonar range device in relation to the centre of the robot
	bool getNextSonarPose(SosPose& pose)
	{
		
		if(_sonarPosesListIterator == _sonarPosesList.end())
		{
			return false;
		}

		pose = *_sonarPosesListIterator;
		_sonarPosesListIterator++;
		return true;
	}

	//returns the pose of a laser range device in relation to the centre of the robot
	bool getNextLaserPose(SosPose& pose)
	{
		
		if(_laserPosesListIterator == _laserPosesList.end())
		{
			return false;
		}

		pose = *_laserPosesListIterator;
		_laserPosesListIterator++;
		return true;
	}

//----------------------------------------------

	//get the next Pose Reading.  Pose readings are where there is no range data, as can be seen in the 
	//"DATA POSE" section in the example at the bottom of the file.
	//Returns false if it has reached the end of the list 
	//(and therefore a valid reading was NOT returned), otherwise returns true
	bool getNextPose(SensorReading& reading)
	{
		
		if(_posesListIterator == _posesList.end())
		{
			return false;
		}

		reading = *_posesListIterator;
		_posesListIterator++;
		return true;
	}
//----------------------------------------------

	//gets the radius of the robot
	long getRobotRadius()
	{
		return _robotRadius;
	}

	//set the robot radius
	void setRobotRadius(long radius)
	{
		_robotRadius = radius;
	}

//----------------------------------------------

	//push a sonar range reading onto the list
	void pushSonarReading(const SensorReading & reading)
	{
		_sonarReadingsList.push_back(reading);
	}

	//push a laser range reading onto the list
	void pushLaserReading(const SensorReading & reading)
	{
		_laserReadingsList.push_back(reading);
	}

	//Push the pose of a sonar onto the list
	void pushSonarPose(const SosPose & pose)
	{
		_sonarPosesList.push_back(pose);
	}

	//push the pose of a laser onto the list
	void pushLaserPose(const SosPose & pose)
	{
		_laserPosesList.push_back(pose);
	}

	//push a pose reading onto the list
	void pushPoseReading(const SensorReading& reading)
	{
		_posesList.push_back(reading);
	}

//--------------------------------------------------

	//write the information stored in the lists and robot radius to a the file "fileName".
	//For sonar data to be written, there must be information both on the positions of the sonars 
	//using the pushSonarPose method, and on the range data from the sonars using the pushSonarReading method.
	//The same goes for laser data.
	//For just Pose data, only Pose data has to be entered, using the pushPoseReading method
	virtual bool writeToFile(char* fileName) = 0;


protected:
	std::list<SensorReading>		_sonarReadingsList;
	std::list<SensorReading>		_laserReadingsList;
	std::list<SensorReading>		_posesList;
	std::list<SosPose>				_sonarPosesList;
	std::list<SosPose>				_laserPosesList;

	std::list<SensorReading>::const_iterator	_sonarReadingsListIterator;
	std::list<SensorReading>::const_iterator	_laserReadingsListIterator;
	std::list<SensorReading>::const_iterator	_posesListIterator;
	std::list<SosPose>::const_iterator			_sonarPosesListIterator;
	std::list<SosPose>::const_iterator			_laserPosesListIterator;
	
	long							_robotRadius;
	char							_errorToken[1024];

	int stringInArrayNoCase(char* str, char** strArray, int arrSize)
	{
		if(str == 0 || strlen(str) == 0 || strArray == 0 || arrSize < 1)
			return -1;

		for(int i = 0; i < arrSize; i++)
		{
			if(stricmp(str,strArray[i]) == 0)
				return i;
		}

		return -1;
	}

	bool is_integer(char * str)
	{
		if ( strspn ( str, "0123456789+-" ) != strlen(str) ) 
			return ( false) ;
		else
			return ( true) ;
	}

	bool is_numeric(char * str) 
	{
		if ( strspn ( str, "0123456789.+-eE" ) != strlen(str) ) 
			return ( false) ;
		else
			return ( true) ;
	}
	DEF_LOG
};




#endif