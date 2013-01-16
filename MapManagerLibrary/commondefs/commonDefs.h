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

#ifndef COMMONDEFS_H
#define COMMONDEFS_H

//NB!!!  Always #include this file before all other files in order for 
//  all #define calls to be completed before being used

#define ROBOT_RADIUS 220
#define SONAR_ANGLE 23
#define MAX_RANGE 2500
#define MIN_RANGE 10

#define SONAR_ERROR 0.02 //need to change this when find out actual value

#define NO_OF_SONARS 7

#define MAXBUF 1024

//these filter variables specify what are the min and max values allowed to 
//be passed to the Mapping.setFilteringLevel method
#define MIN_FILTER 1
#define MAX_FILTER 5

#ifndef M_PI
#define M_PI        3.14159265358979323846
#endif
 

#define XX 0
#define YY 1

#ifndef _GNU_SOURCE 
//define _GNU_SOURCE so that the log2() function will work - otherwise the mathcalls.h file will not define the log2() function
#define _GNU_SOURCE 
#endif



#include <string>
#include "SosUtil.h"
#include "MapUpdateInterface.h"
#include "../logger/Logger.h"

#ifndef HUGENUM
#define HUGENUM (long)101029385
#endif

struct CommandSet
{
    bool saveMap;
    char savePath[256];
    char systemToLoad[64];
    char viewAMap[256];
    bool toggleSonars;
    bool localisationOn;
    bool blurMap;
    bool impMap;
};

template <class T>
struct SensorSet
{  
    int numSonars;
    SosPose* sonars;
    long robotRadius;
    double sonarWidth;
	double leftLineSlope;
  T* clientPointer;
};

class SensorReading
{
public:

	bool operator==(const SensorReading& node) const
	{
		return _numSensors == node._numSensors &&
			_robotPose == node._robotPose;
	}

	bool operator!=(const SensorReading& node) const
	{
		return !(*this == node);

	}

	SensorReading()
	{
		GET_FILE_LOG
		//LOGGING_OFF
		init(1);
	}

	SensorReading(unsigned int numSensors)
	{
		GET_FILE_LOG

	//	LOGGING_OFF
		LOG<<"new SensorReading("<<this<<")";

		init(numSensors);

		LOG<<"At end of SensorReading constructor"<<endl;
	}

	SensorReading(const SensorReading& val)
	{
		GET_FILE_LOG
	//	LOGGING_OFF
		LOG<<"In SensorReading copy constructor"<<endl;
		_ranges = 0;
		*this = val;
	}

	~SensorReading()
	{
	//	LOG<<"~SensorReading("<<this<<") deleting ranges with address "<<_ranges<<endl;
		delete [] _ranges;
	}

	void setPose(SosPose pose)
	{
		_robotPose.setPose(pose);
	}

	void setPose(double x, double y, double angle)
	{
		_robotPose.setPose(x,y,angle);
	}

	const SosPose getPose() const
	{
		return _robotPose;
	}

	void setRange(int rangeNum, long dist)
	{
		if(rangeNum < 0 || rangeNum >= _numSensors)
			return;

		_ranges[rangeNum] = dist;
	}

	void setRange(long* distances)
	{
		LOG<<"Setting Ranges:";
		for(int i = 0; i< _numSensors; i++)
		{
			LOG<<i<<' '<<distances[i];
			_ranges[i] = distances[i];
		}
	}

	const long getRange(int num) const 
	{
		if(num < 0 || num >= _numSensors)
			return 0;

		return _ranges[num];

	}

	const int numSensors() const 
	{
		return (int) _numSensors;
	}

	virtual const SensorReading& operator=(const SensorReading& val)
	{
		LOG<<"In SensorReading& operator=()"<<endl;
		if(_ranges != 0)
		{
			LOG<<"Deleting ranges array"<<endl;
			delete [] _ranges;
		}

		_robotPose = val.getPose();
		_numSensors = val.numSensors();

		LOG<<"Creating a new array of size "<<_numSensors;
		_ranges = new long[_numSensors];

		LOG<<"about to copy ranges with for loop"<<endl;
		for(int i = 0; i< _numSensors; i++)
		{
			_ranges[i] = val.getRange(i);
		}		
		
		return (*this);
	}

protected:
	unsigned int _numSensors;
	long* _ranges;
	SosPose _robotPose;

	void init(unsigned int numSensors)
	{
		if(numSensors <1)
			numSensors = 1;

		_numSensors = numSensors;
		_ranges = new long[_numSensors];

		for(int i = 0; i< _numSensors; i++)
		{
			_ranges[i]	= 0;
		}

		_robotPose.setPose(0,0,0);

	}
	DEF_LOG
};



class SensorReadingSet : public SensorReading
{
public:

//	LOGCODE static long _objectCounter;

	bool operator<(const SensorReadingSet& node) const{return false;}
	bool operator>(const SensorReadingSet& node) const{return false;}
	bool operator==(const SensorReadingSet& node) const
	{
		return _clientNumber == node._clientNumber &&
			_anythingElse == node._anythingElse;

	}

	bool operator!=(const SensorReadingSet& node) const
	{
		return !(*this == node);

	}
	virtual const SensorReadingSet& operator=(const long val)
	{
		_sensorPoses = 0;
    	_clientNumber = 0;
    	_anythingElse = 0; 
		return *this;
	}

	SensorReadingSet(int numSensors = 1) :SensorReading(numSensors)
	{
		GET_FILE_LOG

//		_myNum = _objectCounter++;

		SosPose tempPose(0,0,0);
		_sensorPoses = new SosPose[_numSensors];
		_sonarConfidences = new double[_numSensors];

//		LOG<<"new SensorReadingSet("<<_myNum<<","<<this<<"), _sonarConfidences = "<<_sonarConfidences<<endl;

		for(int i = 0; i< _numSensors; i++)
		{
			_sensorPoses[i]			= tempPose;
			_sonarConfidences[i]	= 0;
		}
	
		_clientNumber = -1;
		_anythingElse = 0;
	}

	SensorReadingSet(SensorReadingSet& reading): SensorReading(1)
	{
		GET_FILE_LOG

//		_myNum = _objectCounter++;

//		LOG<<"In SensorReadingSet copy constructor ("<<_myNum<<")"<<endl;
		_sonarConfidences = 0;
		_sensorPoses = 0;
		*this = reading;
	}

	SensorReadingSet(SensorReading& reading): SensorReading(1)
	{
		GET_FILE_LOG

//		_myNum = _objectCounter++;

//		LOG<<"In SensorReadingSet copy constructor ("<<_myNum<<")"<<endl;
		_sonarConfidences = 0;
		_sensorPoses = 0;
		*this = reading;
	}



	~SensorReadingSet()
	{
	//	_objectCounter--;
//		LOG<<"~SensorReadingSet("<<_myNum<<","<<this<<")going to delete _sonarConfidences, value is "<<_sonarConfidences<<", numSonars = "<<_numSensors<<endl;
		delete [] _sonarConfidences;
		delete []_sensorPoses;
	}

	void setSensorPose(int numSensor,const SosPose& pose)
	{
		if(numSensor < 0 || numSensor >= _numSensors)
			return;

		_sensorPoses[numSensor] = pose;
	}

	SosPose getSensorPose(int numSensor) const
	{
		if(numSensor < 0 || numSensor >= _numSensors)	
		{
			SosPose error;
			return error;
		}

		return _sensorPoses[numSensor];
	}

	void setClientNum(int num)
	{
		_clientNumber = num;
	}

	int getClientNum() const
	{
		return _clientNumber;
	}

	void setSonarConfidance(int numSensor, double val)
	{
		if(numSensor < 0 || numSensor >= _numSensors)
			return;

		_sonarConfidences[numSensor] = val;
	}

	double getSonarConfidance(int numSensor) const
	{
		if(numSensor < 0 || numSensor >= _numSensors)
		{			
			return -1;
		}

		return _sonarConfidences[numSensor];
	}

	void setAnythingElse(double val)
	{
		_anythingElse = val;
	}

	double getAnythingElse() const
	{
		return _anythingElse;
	}

	
	virtual const SensorReadingSet& operator=(const SensorReadingSet& val)
	{	
//		LOG<<_myNum<<":Called SensorReadingSet operator=(const SensorReadingSet val), _sonarConfidences = "<<_sonarConfidences<<endl;
		int i = 0;

		_robotPose = val.getPose();
		_numSensors = val.numSensors();
		_clientNumber = val.getClientNum();
		_anythingElse = val.getAnythingElse();
	
		if(_ranges != 0)
		{
			delete [] _ranges;
			_ranges = 0;
	//		LOG<<"Set _ranges to 0"<<endl;
		}

		_ranges = new long[_numSensors];

		if(_sensorPoses != 0)
		{
			delete []_sensorPoses;
			_sensorPoses = 0;
	//		LOG<<"Set _sensorPoses to 0"<<endl;
		}

		_sensorPoses = new SosPose[_numSensors];

		if(_sonarConfidences != 0)
		{
			delete [] _sonarConfidences;
			_sonarConfidences = 0;
	//		LOG<<"Set _sonarConfidences to 0"<<endl;
		}

		_sonarConfidences = new double[_numSensors];

		for(i = 0; i< _numSensors; i++)
		{
			_ranges[i]				= val.getRange(i);
			_sensorPoses[i]			= val.getSensorPose(i);
			_sonarConfidences[i]	= val.getSonarConfidance(i);
		}	
	
	//	LOG<<"At end of SensorReadingSet::operator=(), _sonarConfidences = "<<_sonarConfidences<<endl;
		
		return *this;
	}

	virtual const SensorReading& operator=(const SensorReading& val)
	{
	//	LOG<<_myNum<<":Called SensorReading operator=(const SensorReading val)";
		
		SosPose tempPose(0,0,0);

		_robotPose = val.getPose();
		_numSensors = val.numSensors();

		if(_ranges != 0)
		{
			delete [] _ranges;
			_ranges = 0;
	//		LOG<<"Set _ranges to 0"<<endl;
		}

		_ranges = new long[_numSensors];

		if(_sensorPoses != 0)
		{
			delete []_sensorPoses;
			_sensorPoses = 0;
	//		LOG<<"Set _sensorPoses to 0"<<endl;
		}

		_sensorPoses = new SosPose[_numSensors];

		if(_sonarConfidences != 0)
		{
			delete [] _sonarConfidences;
			_sonarConfidences = 0;
	//		LOG<<"Set _sonarConfidences to 0"<<endl;
		}

		_sonarConfidences = new double[_numSensors];

		for(int i = 0; i< _numSensors; i++)
		{
			_ranges[i]				= val.getRange(i);
			_sensorPoses[i]			= tempPose;
			_sonarConfidences[i]	= 0;
		}	

		return *this;
	}

	double *		_sonarConfidences;
private:	
    SosPose*		_sensorPoses;
    int				_clientNumber;
    double			_anythingElse; 

	DEF_LOG
};


//long SensorReadingSet::_objectCounter = 0;

class SosListPos
{
public:
  typedef enum {
      FIRST = 1, ///< place item first in the list
      LAST = 2 ///< place item last in the list
  } Pos;
};

struct StageWorldOptions
{
	bool	hasSonar;
	bool	hasLaser;	
	int		initialPort;
};

class WorldOptions
{
public:
	WorldOptions()
	{
		numRooms = 1;
		avgRoomArea = 10.0f;//default of 10m squared
		createObstacles = true;
		useChair = false;
		useDiningArea = false;
		useBox = true;
		percentObstacleArea = 25.0;//default 25% of room covered in obstacles
		useRoomGridLayout = true;
		fileType = -1; //
		numMaps = 1;
		numRobots = 1;
	}
	int numRooms;
	float avgRoomArea;
	bool createObstacles;
	bool useRoomGridLayout;
	bool useChair, useDiningArea,useBox;

	double percentObstacleArea;
	int fileType;
	long numMaps;
	long numRobots;
};

#endif
