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

#undef DEF_LOG	
#undef GET_FILE_LOG			
#undef ENTRYEXIT_LOG_ON		
#undef ENTRYEXIT_LOG_OFF		
#undef LOGGING_OFF				
#undef GET_SCREEN_LOG			
#undef GET_FILE_LOG_GLOBAL		
#undef GET_SCREEN_LOG_GLOBAL   
#undef LOGGING_OFF_GLOBAL		
#undef LOG						
#undef LOGTIME					
#undef LOGENTRY				
#undef LOGEXIT				
#undef LOGCODE					
#undef LOGCODEEND				
#undef LOGCODESTART	

#include <iostream.h>
#include <fstream.h>
#include <string.h>
#include <time.h>
#include <assert.h>

#ifndef LOGGERSOS_H
inline char* stripPath(char*);
inline void replaceChar(char* , char, char);
char* dirname(char* str, char* buffer);
#endif

#ifndef LOG_PATH
#define LOG_PATH "c:\\"
#endif

#ifndef LOGGINGENABLED 
	#define LOGGINGENABLED 0
#endif

//To disable logging, set the value below to 0.  To enable it, set it to 1
#if LOGGINGENABLED
//if logging from within a class, use the following logging macros

	//place this in the private section of class definition
	#define DEF_LOG					ostream* outstr;ostream* entryexit;
	//place this in all the class constructors
	#define GET_FILE_LOG			outstr = Logger::getFileOstream();
	#define ENTRYEXIT_LOG_ON		entryexit = outstr;
	#define ENTRYEXIT_LOG_OFF		entryexit = Logger::getNullOutput();
	//place in class constructor to turn off logging for that class
	#define LOGGING_OFF				outstr = Logger::getNullOutput();//(ostream*)new ofstream();
	#define GET_SCREEN_LOG			outstr = Logger::getScreenOstream();

	//if logging in a standalone function, use these macros

	//place this in the global space of the file
	#define GET_FILE_LOG_GLOBAL		ostream* outstr = Logger::getFileOstream();
	#define GET_SCREEN_LOG_GLOBAL   ostream* outstr = Logger::getScreenOstream();
	#define LOGGING_OFF_GLOBAL		ostream* outstr = Logger::getNullOutput();//(ostream*)new ofstream();


	//this can be used anywhere. e.g.  LOG<<"this"<<"is"<<endl<<"a log entry";
	#define LOG     *outstr<<endl<<stripPath(__FILE__) <<"("<< __LINE__<<") "
	#define LOGTIME {const long t = time(0);*outstr<<endl<<stripPath(__FILE__) <<"("<< __LINE__<<") "<<ctime(&t)<<endl;}
	#define LOGENTRY(a)  *entryexit<<endl<<stripPath(__FILE__) <<"("<< __LINE__<<") " ## <<"<ENTRY>"<< ## a <<endl;
	#define LOGEXIT(a)    *entryexit<<endl<<stripPath(__FILE__) <<"("<< __LINE__<<") " ## <<"<EXIT>"<< ## a<<endl ;
	#define LOGCODE		  
	#define LOGCODESTART  {
	#define LOGCODEEND    }

#else
	#include "LoggerOff.h"
#endif

#ifndef LOGGERSOS_H
#define LOGGERSOS_H

class Logger
{
	public:
		~Logger();
		static ostream* getFileOstream();
		static ostream* getScreenOstream();	
		static ostream* getNullOutput();
		
	private:
		Logger(ostream*);//{_outstr = 0;}	
		static ofstream * _outstr;
		static ofstream * _nullOutput;
};

class NullOutput: public ostream
{
public:
	NullOutput(){}
	NullOutput(streambuf*){}
	virtual ~NullOutput(){}

        NullOutput& flush(){}
inline  int  opfx(){}
inline  void osfx(){}

inline  void operator<<(ostream& (__cdecl * _f)(ostream&)){}
inline  void operator<<(ios& (__cdecl * _f)(ios&)){}
inline  void operator<<(const char *c){}
inline  void operator<<(const unsigned char *){}
inline  void operator<<(const signed char *){}
inline  void operator<<(char){}
inline  void operator<<(unsigned char){}
inline  void operator<<(signed char){}
inline  void operator<<(short){}
inline  void operator<<(unsigned short){}
inline  void operator<<(int){}
inline  void operator<<(unsigned int){}
inline  void operator<<(long){}
inline  void operator<<(unsigned long){}
inline  void operator<<(float){}
inline  void operator<<(double){}
inline  void operator<<(long double){}
inline  void operator<<(const void *){}
inline  void operator<<(streambuf*){}
inline  void put(char){}
inline  void put(unsigned char){}
inline  void put(signed char){}
inline  void write(const char *,int){}
inline  void write(const unsigned char *,int){}
inline  void write(const signed char *,int){}
inline  void seekp(streampos){}
inline  void seekp(streamoff,ios::seek_dir){}
inline  streampos tellp(){}
};

char* stripPath(char* str)
{
	if(str == 0)
		return "";

	long i = strlen(str) - 1;

	//search backwards - probably faster
	for(;i>= 0;i--)
	{
		if(str[i]=='\\' || str[i] =='/')
			return str+i+1;
	}

	return str;
}

#endif