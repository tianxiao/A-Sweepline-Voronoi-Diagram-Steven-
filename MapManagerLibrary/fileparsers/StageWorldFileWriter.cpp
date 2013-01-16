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

#include "StageWorldFileWriter.h"
#include <fstream.h>

StageWorldFileWriter::StageWorldFileWriter()
{
	_initialPort = 6665;
	_robotList = 0;
	_useLaser = _useSonar = false;
	_resolution = 0.01f;
	_translationX = _translationY = 0;

}

StageWorldFileWriter::~StageWorldFileWriter()
{


}
	
void StageWorldFileWriter::setRobots(List<LineXYLayer>* robotList)
{
	_robotList = robotList;

}

void StageWorldFileWriter::setInitialPort(int port)
{
	_initialPort = port;
}

void StageWorldFileWriter::setUseSonar(bool val)
{
	_useSonar = val;
}
	
void StageWorldFileWriter::setUseLaser(bool val)
{
	_useLaser = val;
}

bool StageWorldFileWriter::save(char* worldFileName, char* pnmFileName)
{
	if(worldFileName == 0 || pnmFileName == 0)
	{
		return false;
	}

	ofstream out;
	out.open(worldFileName, ios::out | ios::binary);

	if(!out.is_open())
		return false;

	out<<"# "<<SosUtil::stripPath(worldFileName)<<"\n";
	out<<"# Created by MapViewer - http://mapviewer.skynet.ie\n";

	out<<"\nunit_length \"mm\""<<"\n";

	
	out<<"\nresolution "<<_resolution<<"\n";

	out<<"\nbitmap\n(\n\tfile \""<<SosUtil::stripPath(pnmFileName)<<"\"\n\tresolution "
			<<_resolution<<"\n)";
	
	if(_robotList == 0 || _robotList->getListSize() == 0)
	{
		out.close();
		return true;
	}

	out<<"\n\ndefine mobot position\n(\n\t";

	if(_useSonar)
	{
		out<<"sonar()\n\t";
	}
	if(_useLaser)
	{
		out<<"laser()\n\t";
	}

	out<<"shape \"circle\"\n\t";

	LineXYLayer rob ;
	
	if(!_robotList->readHead(rob))
	{
		out.close();
		return true;
	}

	out<<"size ["<<SosUtil::setPrecision(SosUtil::fabs(rob.pt1.x - rob.pt2.x),3)
		<<" "<<SosUtil::setPrecision(SosUtil::fabs(rob.pt1.y - rob.pt2.y),3)
		<<"]";

	out<<"\n)\n";

	_robotList->resetIterator();

	float x = 0, y = 0, angle = 0;
	int port = _initialPort;

	while(_robotList->readNext(rob))
	{
		angle = rob.value;
		x = SosUtil::midWay(rob.pt1.x,rob.pt2.x);
		y = SosUtil::midWay(rob.pt1.y,rob.pt2.y);

		out<<"\nmobot ( pose ["<<SosUtil::setPrecision(x,3)<<" "
			<<SosUtil::setPrecision(y,3)<<" "<<angle<<"] port "<<port<<")";
		port++;
	}
	

	out.close();

	return true;
}

void StageWorldFileWriter::setResolution(float val)
{
	_resolution = val;
}