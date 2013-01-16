#ifndef THREADEDIMPL_H
#define THREADEDIMPL_H

#include "../sosutil/Threaded.h"
#include "../mapbuilding/mapping/Mapping.h"
#include "../fileparsers/RobotRunFileHelper.h"
#include "../logger/Logger.h"
#include "JobProgressMonitor.h"


class ThreadedMapBuilder : public Threaded
{
public:
	ThreadedMapBuilder(RobotRunFileHelper * parser, Mapping* mapSystem,JobProgressMonitor* monitor = 0)
	{
		GET_FILE_LOG
		_parser = parser;
		_mapSystem = mapSystem;
		_monitor = monitor;
	}

	virtual void run()
	{
		LOG<<"In run()";
		if(_parser == 0 || _mapSystem == 0)
			return;

		SensorReading reading(_parser->numSonars());
		double totalReadings = _parser->numSonarReadings();
		double readingsProcessed = 0, perc = 0;
		long percLong = 0;

		LOG<<"_monitor = "<<endl;
		LOG<<"Total Num Readings = "<<totalReadings;
		if(_monitor == 0)
		{
			while(_isRunning && _parser->popSonarReading(reading) )
			{					
				_mapSystem->pushSonar(reading, 0, 1);						
			}
		}
		else
		{
			while(_isRunning && _parser->popSonarReading(reading))
			{	
				_mapSystem->pushSonar(reading, 0, 1);

				readingsProcessed++;

				perc = (readingsProcessed*100)/totalReadings;

				if((long)perc > percLong)
				{
					percLong = (long)perc;
					_monitor->setPercentageComplete(perc);
					
					LOG<<"Setting percentage complete to "<<perc;
				}								
			}
		}
		if(_isRunning)
		{
			_monitor->setFinished();
		}
		threadFinished();

	}


private:
	RobotRunFileHelper*		_parser;
	JobProgressMonitor*		_monitor;
	Mapping*				_mapSystem;

	DEF_LOG
};


#endif