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

#ifndef STAGEWORLDFILEWRITER_H
#define STAGEWORLDFILEWRITER_H


#include "SosList.h"
#include "SosUtil.h"
#include "../logger/LoggerOff.h"

class StageWorldFileWriter
{
public:
	StageWorldFileWriter();
	~StageWorldFileWriter();

	
	void setRobots(List<LineXYLayer>* robotList);
	void setInitialPort(int port);
	void setUseSonar(bool val);
	void setUseLaser(bool val);
	void setResolution(float val);
	void setTranslation(float x, float y);

	bool save(char* fileName, char* pnmFileName);


private:

	List<LineXYLayer>*	_robotList;
	int					_initialPort;
	bool				_useSonar, _useLaser;
	float				_resolution;
	float				_translationX, _translationY;


};

#endif