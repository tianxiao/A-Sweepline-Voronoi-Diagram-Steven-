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


#ifndef STAGEWORLDFILEPARSER_HH
#define STAGEWORLDFILEPARSER_HH

#include <stdio.h>
#include "commonDefs.h"
#include "SosVector.h"
#include "SosUtil.h"
#include "../logger/Logger.h"

class StageItem
{
public:
	StageItem()
	{
		type = 0;
		value = 0;
	}
	char *value;
	int type;
	
};

class StageMacro
{
public:
	StageMacro()
	{
		macroname = entityname = 0;
		line = startitem = 0;
	}
	const char *macroname;
	const char *entityname;
	int line;
	int startitem;
};

struct StageEntity
{
	StageEntity(){parent = 0; type = 0;}
	int parent;
	const char *type;
};

struct StageProperty
{
	StageProperty()
	{
		entity = value_count = line = 0;
		name = 0;
		values = 0;
		used = false;
	}
	int entity;
	const char *name;
	int value_count;
	int *values;
	int line;
	bool used;
};


class StageWorldFileParser
{
public: 
	StageWorldFileParser();
	~StageWorldFileParser();
	
	bool loadFile(const char *filename);
		
	const char *getStr(int entity, const char *name, const char *value);
		
	double getDbl(int entity, const char *name, double value);
		
	double getDbl(int entity, const char *name,	int index, double value);
	
	int numEntities();

	const char *entityType(int entity);
	
	int findEntity(const char *type);
	
private: 
	bool loadFile(int include);
		 
	bool processComment(int *line, int include);
		 
	bool processWord(int *line, int include);
		 
	bool processInclude(int *line, int include);
		 
	bool processNum(int *line, int include);
		 
	bool processString(int *line, int include);
		 
	bool processSpace(int *line, int include);
		 
	void resetItems();
		 
	bool pushItem(int type, const char *value);
		 
	bool setValue(int index, const char *value);
		 
	const char *getValue(int index);
		 
	bool processFile();
		 
	bool parseInclude(int *index, int *line);
		 
	bool parseDefine(int *index, int *line);
		 
	bool parseWord(int entity, int *index, int *line);
		 
	bool parseEntity(int entity, int *index, int *line);
		 
	bool parseProperty(int entity, int *index, int *line);
		 
	bool parseTuple(int entity, int property, int *index, int *line);
		 
	int pushMacro(const char *macroname, const char *entityname, int line, int startitem);
		 
	void resetEntities();
		 
	int pushEntity(int parent, const char *type);
		
	void resetProperties();
		 
	int pushProperty(int entity, const char *name, int line);
		 
	void pushPropertyValue(int property, int index, int value_item);
		 
	int getProperty(int entity, const char *name);
		 
	const char *getPropertyValue(int property, int index);
		 
	enum
	{
		TokenComment,
		TokenWord, TokenNum, TokenString,
		TokenOpenEntity, TokenCloseEntity,
		TokenOpenTuple, TokenCloseTuple,
		TokenSpace, TokenEOL
	};	
		 
	Vector<StageItem>* _items;

	Vector<StageMacro>* _macros;

	Vector<StageEntity>* _entities;

	Vector<StageProperty>* _properties;
		
	char *_filename;
	FILE *_openFile;
		 
	DEF_LOG
};

#endif
