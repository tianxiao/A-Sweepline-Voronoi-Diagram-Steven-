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

#ifndef LOGGER_CPP
#define LOGGER_CPP

#include "Logger.h"
#include <assert.h>

ofstream * Logger::_outstr = 0;
ofstream * Logger::_nullOutput = 0;

Logger::~Logger()
{
	const long t = time(0);
	*_outstr<<"\n--------------------------------------------------"
			<<"\nFile closed on "<<ctime(&t)
			<<"--------------------------------------------------";
	(*_outstr).close();

	if(_nullOutput != 0)
		delete _nullOutput;
}

ostream* Logger::getScreenOstream()
{
	return &cout;
}

ostream* Logger::getFileOstream()
{
	if(_outstr==0)
	{
		const long t = time(0);	
		char filename[256] = LOG_PATH;

		assert(strlen(filename) != 0);		

		strcat(filename,"log");
		
		strcat(filename,".txt");
		
		//cout<<filename<<endl;
		_outstr=new ofstream(filename, ios::out);
		
		*_outstr<<"\n--------------------------------------------------"
			<<"\nFile opened on "<<ctime(&t)
			<<"--------------------------------------------------"<<endl;

	}

	return _outstr;

}

ostream* Logger::getNullOutput()
{
	if(_nullOutput == 0)
	{
		_nullOutput = new ofstream();
	}
	return _nullOutput;
}
		
#endif
void replaceChar(char* str, char oldChar, char newChar)
{
	if(str == 0)
		return;

	long len=strlen(str);
	for(int x = 0; x<len;x++)
	{
		if(str[x] == oldChar)
			str[x] = newChar;
	}

}


char* dirname(char* str, char* buffer)
{
	if(str == 0)
		return "";

	long i = strlen(str) - 1;

	//search backwards - probably faster
	for(;i>= 0;i--)
	{
		if(str[i]=='\\' || str[i] =='/')
			break;
			//return str+i+1;
	}

	if(i > 0)
	{
		buffer = strdup(str);
		buffer[i] = '\0';
	}

	return buffer;
}