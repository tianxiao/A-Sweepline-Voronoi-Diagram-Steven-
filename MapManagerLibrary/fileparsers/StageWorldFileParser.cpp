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


#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <locale.h>

//#include "stage_types.h"
#include "StageWorldFileParser.h"



///////////////////////////////////////////////////////////////////////////
// Default constructor
StageWorldFileParser::StageWorldFileParser() 
{
	GET_FILE_LOG
//	LOGGING_OFF		
	LOG<<"In StageWorldFileParser constructor";
	
	_filename = 0;
	_openFile = 0;

	StageItem item;
	_items = new Vector<StageItem>(item);
	
	StageMacro macro;	
	_macros = new Vector<StageMacro>(macro);

	StageEntity entity;
	_entities = new Vector<StageEntity>(entity);

	StageProperty prop;
	_properties = new Vector<StageProperty>(prop);
}


///////////////////////////////////////////////////////////////////////////
// Destructor
StageWorldFileParser::~StageWorldFileParser()
{
	resetProperties();
	
	
	delete _macros;
	_macros = 0;

	resetEntities();
	resetItems();
	
	delete _items;
	delete _entities;

	if (_filename != 0)
		free(_filename);
}


///////////////////////////////////////////////////////////////////////////
// Load world from file
bool StageWorldFileParser::loadFile(const char *filePath)
{
	LOG<<"In StageWorldFileParser::loadFile"<<endl;
	LOG<<endl;
	if(filePath== 0)
	{
		LOG<<"Filepath is null, returning false"<<endl;
		return false;
	}

	LOG<<"in StageWorldFileParser::Load()"<<endl;
//	LOG<<"filePath = "<<filePath<<endl;

	if(_openFile != 0)
	{
		LOG<<"closing previous file"<<endl;
		fclose(_openFile);
		_openFile = 0;
	}

	LOG<<"About to copy the filePath"<<endl;
	_filename = strdup(filePath);
	
	_openFile = fopen(_filename, "r");
	if (_openFile == 0)
	{		
		LOG<<"Could not open file "<<_filename;
		return false;
	}
	
	resetItems();
	
	LOG<<"About to read the _items from the file"<<endl;
	// Read _items from the file
	if (!loadFile(0))
	{		
		LOG<<"LoadFile(0) returned false, returning false";
		return false;
	}

	LOG<<"About to parse the _items"<<endl;
	
	// Parse the _items to identify _entities
	if (!processFile())
	{
		LOG<<"ProcessFile() returned false, returning false";
		return false;
	}

	LOG<<"After processing the file"<<endl;
	// Dump contents and exit if this file is meant for debugging only.
	if (getStr(0, "test", 0) != 0)
	{
		LOG<<"Exiting because this is a test file"<<endl;
		return false;
	}
		
	return true;
}


///////////////////////////////////////////////////////////////////////////
// Load _items from a file.
bool StageWorldFileParser::loadFile(int include)
{
	LOG<<"In loadFile"<<endl;
	int ch;
	int line;
	char item[256];
	
	line = 1;
	
	while (true)
	{
		ch = fgetc(_openFile);
		if (ch == EOF)
			break;
		
		if ((char) ch == '#')
		{
			ungetc(ch, _openFile);
			if (!processComment(&line, include))
				return false;
		}
		else if (isalpha(ch))
		{
			ungetc(ch, _openFile);
			if (!processWord(&line, include))
				return false;
		}
		else if (strchr("+-.0123456789", ch))
		{
			ungetc(ch, _openFile);
			if (!processNum(&line, include))
				return false;
		}
		else if (ch == ' ' || ch == '\t')
		{
			ungetc(ch, _openFile);
			if (!processSpace(&line, include))
				return false;
		}
		else if (ch == '"')
		{
			ungetc(ch, _openFile);
			if (!processString(&line, include))
				return false;
		}
		else if (strchr("(", ch))
		{
			item[0] = ch;
			item[1] = 0;
			pushItem(TokenOpenEntity, item);
		}
		else if (strchr(")", ch))
		{
			item[0] = ch;
			item[1] = 0;
			pushItem(TokenCloseEntity, item);
		}
		else if (strchr("[", ch))
		{
			item[0] = ch;
			item[1] = 0;
			pushItem(TokenOpenTuple, item);
		}
		else if (strchr("]", ch))
		{
			item[0] = ch;
			item[1] = 0;
			pushItem(TokenCloseTuple, item);
		}
		else if (ch == '\n')
		{
			line++;
			pushItem(TokenEOL, "\n");
		}
		else
		{			
			return false;
		}
	}
	
	return true;
}


///////////////////////////////////////////////////////////////////////////
// Read in a comment item
bool StageWorldFileParser::processComment(int *line, int include)
{
	LOG<<"In processComment"<<endl;
	char item[256];
	int len;
	int ch;
	
	len = 0;
	memset(item, 0, sizeof(item));
	
	while (true)
	{
		ch = fgetc(_openFile);

		if (ch == EOF)
		{
			pushItem(TokenComment, item);
			return true;
		}
		else if (ch == '\n')
		{
			ungetc(ch, _openFile);
			pushItem(TokenComment, item);
			return true;
		}
		else if(len < 256)
		{
			item[len++] = ch;
		}
	}
	return true;
}


///////////////////////////////////////////////////////////////////////////
// Read in a word item
bool StageWorldFileParser::processWord(int *line, int include)
{
	char item[256];
	int len;
	int ch;
	
	len = 0;
	memset(item, 0, sizeof(item));
	
	while (true)
	{
		ch = fgetc(_openFile);
		
		if (ch == EOF)
		{
			pushItem(TokenWord, item);
			return true;
		}
		else if (isalpha(ch) || isdigit(ch) || strchr(".-_[]", ch))
		{
			item[len++] = ch;
		}
		else
		{
			if (stricmp(item, "include") == 0)
			{
				ungetc(ch, _openFile);
				pushItem(TokenWord, item);
				if (!processInclude(line, include))
					return false;
			}
			else
			{
				ungetc(ch, _openFile);
				pushItem(TokenWord, item);
			}
			return true;
		}
	}
	assert(false);
	return false;
}


///////////////////////////////////////////////////////////////////////////
// Load an include item; this will load the include file.
bool StageWorldFileParser::processInclude(int *line, int include)
{
	LOG<<"In processInclude "<<line;
	int ch;
	const char *filename= 0;
	char *fullpath= 0;
	char * buffer = 0;
	
	ch = fgetc(_openFile);

	if (ch == EOF)
	{
		//LOG<<"incomplete include statement"<< *line;
		return false;
	}
	else if (!(ch == ' ' || ch == '\t'))
	{
		//LOG<<"syntax error in include statement"<< *line;
		return false;
	}
	
	ungetc(ch, _openFile);
	if (!processSpace(line, include))
		return false;
	
	ch = fgetc(_openFile);
	
	if (ch == EOF)
	{
		return false;
	}
	else if (ch != '"')
	{
		return false;
	}
	
	ungetc(ch, _openFile);
	if (!processString(line, include))
		return false;
	
	// This is the basic filename
	filename = getValue(_items->size() - 1);
	
	char *tmp = strdup(_filename);
	fullpath = (char*) malloc(256);
	
	SosUtil::getPath(tmp,fullpath);
	
	strcat( fullpath, "/" ); 
	strcat( fullpath, filename );

	assert(strlen(fullpath) + 1 < 256);
	
	free(tmp);		
	
	fflush( stdout );
	
	FILE *infile = fopen(fullpath, "r");
	if (infile == 0)
	{
		free(fullpath);
		return false;
	}

	FILE* oldFile = _openFile;
	_openFile = infile;

	// Terminate the include line
	pushItem(TokenEOL, "\n");
	
	// Read _items from the file
	if (!loadFile(include + 1))
	{
		free(fullpath);
		return false;
	}

	fclose(infile);

	_openFile = oldFile;
	
	free(fullpath);
	return true;
}


///////////////////////////////////////////////////////////////////////////
// Read in a number item
bool StageWorldFileParser::processNum(int *line, int include)
{
	LOG<<"In processNum "<<line;
	char item[256];
	int len;
	int ch;
	
	len = 0;
	memset(item, 0, sizeof(item));
	
	while (true)
	{
		ch = fgetc(_openFile);
		
		if (ch == EOF)
		{
			pushItem(TokenNum, item);
			return true;
		}
		else if (strchr("+-.0123456789", ch))
		{
			item[len++] = ch;
		}
		else
		{
			pushItem(TokenNum, item);
			ungetc(ch, _openFile);
			return true;
		}
	}
	assert(false);
	return false;
}


///////////////////////////////////////////////////////////////////////////
// Read in a string item
bool StageWorldFileParser::processString(int *line, int include)
{
	LOG<<"In processString "<<line<<endl;
	int ch;
	int len;
	char item[256];
	
	len = 0;
	memset(item, 0, sizeof(item));
	
	ch = fgetc(_openFile);
	
	while (true)
	{
		ch = fgetc(_openFile);
		//LOG<<"ch = "<<(char)ch<<" = "<<ch;		
		if (ch == EOF || ch == '\n')
		{
			//LOG<<"unterminated string constant"<<*line;
			return false;
		}
		else if (ch == '"')
		{
			//LOG<<"About to call pushItem with value = "<<item;
			pushItem(TokenString, item);
			return true;
		}
		else if(len < 256)
		{
			item[len++] = ch;
		}
	}

	return false;
}


///////////////////////////////////////////////////////////////////////////
// Read in a whitespace item
bool StageWorldFileParser::processSpace(int *line, int include)
{
	int ch;
	int len;
	char item[256];
	
	len = 0;
	memset(item, 0, sizeof(item));
	
	while (true)
	{
		ch = fgetc(_openFile);
		
		if (ch == EOF)
		{
			pushItem(TokenSpace, item);
			return true;
		}
		else if (ch == ' ' || ch == '\t')
		{
			item[len++] = ch;
		}
		else
		{
			pushItem(TokenSpace, item);
			ungetc(ch, _openFile);
			return true;
		}
	}

	return false;
}


///////////////////////////////////////////////////////////////////////////
// reset the item list
void StageWorldFileParser::resetItems()
{
	LOG<<"In resetItems"<<endl;

	long size = _items->size();
	StageItem item;

	for(int i = 0; i< size; i++)
	{
		item = _items->get(i);
		if(item.value != 0)
			free(item.value);
	}

	_items->clear();
}


///////////////////////////////////////////////////////////////////////////
// push a item to the item list
bool StageWorldFileParser::pushItem(int type, const char *value)
{
	StageItem item;

	item.type = type;
	item.value = strdup(value);

	LOG<<"Pushed item with type "<<type<<" into position "<<_items->size()+1;
	_items->push_back(item);

	return true;
}


///////////////////////////////////////////////////////////////////////////
// Set a item value in the item list
bool StageWorldFileParser::setValue(int index, const char *value)
{
	if(index < 0 || index > _items->size())
		return false;

	StageItem item = _items->get(index);

	free(item.value);
	item.value = strdup(value);
	_items->put(index,item);
	
	return true;
}


///////////////////////////////////////////////////////////////////////////
// Get the value of a item
const char *StageWorldFileParser::getValue(int index)
{
	if(index < 0 || index > _items->size())
		return false;

	StageItem item = _items->get(index);
	
	return item.value;
}



///////////////////////////////////////////////////////////////////////////
// Parse _items into _entities and _properties.
bool StageWorldFileParser::processFile()
{
	LOG<<"In processFile()"<<endl;
	int i;
	int entity;
	int line;
	const StageItem *item = 0;
	
	resetEntities();
	resetProperties();
	
	// push in the "global" entity.
	entity = pushEntity(-1, "");
	line = 1;

	LOG<<"About to go through the _items"<<endl;
	
	for (i = 0; i < _items->size(); i++)
	{
		item = _items->getRef(i);//this->_items + i;
		
		LOG<<"Item "<<i<<" -> "<<item->value;

		switch (item->type)
		{
		case TokenWord:
			if (stricmp(item->value, "include") == 0)
			{
				if (!parseInclude(&i, &line))
				{
					LOG<<"parseInclude returned false";
					return false;
				}
			}
			else if (stricmp(item->value, "define") == 0)
			{
				if (!parseDefine(&i, &line))
				{
					LOG<<"parseDefine returned false";
					return false;
				}
			}
			else
			{
				if (!parseWord(entity, &i, &line))
				{
					LOG<<"parseWord returned false";
					return false;
				}
			}
			break;
		case TokenComment:
			break;
		case TokenSpace:
			break;
		case TokenEOL:
			line++;
			break;
		default:
			LOG<<"Got processFile got default type, returning false";
			return false;
		}
	}
	LOG<<"At end of processFile()"<<endl;
	return true;
}


///////////////////////////////////////////////////////////////////////////
// Parse an include statement
bool StageWorldFileParser::parseInclude(int *index, int *line)
{
	LOG<<"In parseInclude"<<endl;
	int i;
	const StageItem *item;
	
	for (i = *index + 1; i < _items->size(); i++)
	{
		item = _items->getRef(i);//this->_items + i;
		
		switch (item->type)
		{
		case TokenString:
			break;
		case TokenSpace:
			break;
		case TokenEOL:
			*index = i;
			(*line)++;
			return true;
		default:
			return false;
		}
	}
	return false;
}


///////////////////////////////////////////////////////////////////////////
// Parse a macro definition
bool StageWorldFileParser::parseDefine(int *index, int *line)
{
	int i;
	int count;
	const char *macroname, *entityname;
	int startitem;
	const StageItem *item;
	
	count = 0;
	macroname = NULL;
	entityname = NULL;
	startitem = -1;
	
	for (i = *index + 1; i < _items->size(); i++)
	{
		item = _items->getRef(i);//this->_items + i;
		
		switch (item->type)
		{
		case TokenWord:
			if (count == 0)
			{
				if (macroname == NULL)
					macroname = getValue(i);
				else if (entityname == NULL)
				{
					entityname = getValue(i);
					startitem = i;
				}
				else
				{
					return false;
				}
			}
			else
			{
				if (macroname == NULL)
				{
					return false;
				}
				if (entityname == NULL)
				{
					return false;
				}
			}
			break;
		case TokenOpenEntity:
			count++;
			break;
		case TokenCloseEntity:
			count--;
			if (count == 0)
			{
				pushMacro(macroname, entityname, *line, startitem);
				*index = i;
				return true;
			}
			if (count < 0)
			{
				return false;
			}
			break;
		default:
			break;
		}
	}
	return false;
}


///////////////////////////////////////////////////////////////////////////
// Parse something starting with a word; could be a entity or an property.
bool StageWorldFileParser::parseWord(int entity, int *index, int *line)
{
	int i;
	const StageItem *item;
	
	for (i = *index + 1; i < _items->size(); i++)
	{
		item = _items->getRef(i);//this->_items + i;
		
		switch (item->type)
		{
		case TokenComment:
			break;
		case TokenSpace:
			break;
		case TokenEOL:
			(*line)++;
			break;
		case TokenOpenEntity:
			return parseEntity(entity, index, line);
		case TokenNum:
		case TokenString:
		case TokenOpenTuple:
			return parseProperty(entity, index, line);
		default:
			LOG<<"parseWord returning false because the item type is "<<item->type;
			return false;
		}
	}

	LOG<<"At end of parseWord"<<endl;
	
	return false;
}


///////////////////////////////////////////////////////////////////////////
// Parse a entity from the item list.
bool StageWorldFileParser::parseEntity(int entity, int *index, int *line)
{
	LOG<<"In parseEntity"<<endl;
	int i = 0;
	int macro = -1;
	int name;
	const StageItem *item;
	
	name = *index;

	const StageMacro *macroObj;
	const char* macroname = getValue(name);

	LOG<<"Looking for macro name "<<macroname<<" in macro Vector of size "<<_macros->size();
	
	for (i = 0; i < _macros->size(); i++)
	{
		macroObj = _macros->getRef(i);
		if (stricmp(macroObj->macroname, macroname) == 0)
		{
			macro = i;
			break;
		}
	}

	LOG<<"macro = "<<macro;

	// If the entity name is a macro...
	if (macro >= 0)
	{
		const StageMacro *macroObj = _macros->getRef(macro);
		// This is a bit of a hack
		int nentity = _entities->size();
		int mindex = macroObj->startitem;
		int mline = macroObj->line;
		if (!parseEntity(entity, &mindex, &mline))
			return false;
		entity = nentity;
		
		for (i = *index + 1; i < _items->size(); i++)
		{
			item = _items->getRef(i);
			
			switch (item->type)
			{
			case TokenOpenEntity:
				break;
			case TokenWord:
				if (!parseWord(entity, &i, line))
					return false;
				break;
			case TokenCloseEntity:
				*index = i;
				return true;
			case TokenComment:
				break;
			case TokenSpace:
				break;
			case TokenEOL:
				(*line)++;
				break;
			default:
				LOG<<"parseEntity returning false 1 because item->type = "<<item->type<<" on item "<<i;
				return false;
			}
		}
		
	}
	
	// If the entity name is not a macro...
	else
	{
		for (i = *index + 1; i < _items->size(); i++)
		{
			item = _items->getRef(i);
			
			switch (item->type)
			{
			case TokenOpenEntity:
				entity = pushEntity(entity, getValue(name));
				break;
			case TokenWord:
				if (!parseWord(entity, &i, line))
				{
					LOG<<"parseEntity returning false 2";
					return false;
				}
				break;
			case TokenCloseEntity:
				*index = i;
				return true;
			case TokenComment:
				break;
			case TokenSpace:
				break;
			case TokenEOL:
				(*line)++;
				break;
			default:
				LOG<<"parseEntity returning false 3";
				return false;
			}
		}
		
	}
	LOG<<"parseEntity returning false 4";
	return false;
}


///////////////////////////////////////////////////////////////////////////
// Parse an property from the item list.
bool StageWorldFileParser::parseProperty(int entity, int *index, int *line)
{
	LOG<<"In parseProperty"<<endl;
	int i, property;
	int name, value, count;
	const StageItem *item;
	
	name = *index;
	value = -1;
	count = 0;
	
	for (i = *index + 1; i < _items->size(); i++)
	{
		item = _items->getRef(i);
		
		switch (item->type)
		{
		case TokenNum:
			property = pushProperty(entity, getValue(name), *line);
			pushPropertyValue(property, 0, i);
			*index = i;
			return true;
		case TokenString:
			property = pushProperty(entity, getValue(name), *line);
			pushPropertyValue(property, 0, i);
			*index = i;
			return true;
		case TokenOpenTuple:
			property = pushProperty(entity, getValue(name), *line);
			if (!parseTuple(entity, property, &i, line))
				return false;
			*index = i;
			return true;
		case TokenSpace:
			break;
		default:
			return false;
		}
	}
	return true;
}


///////////////////////////////////////////////////////////////////////////
// Parse a tuple.
bool StageWorldFileParser::parseTuple(int entity, int property, int *index, int *line)
{
	int i, count;
	const StageItem *item;
	
	count = 0;
	
	for (i = *index + 1; i < _items->size(); i++)
	{
		item = _items->getRef(i);
		
		switch (item->type)
		{
		case TokenNum:
			pushPropertyValue(property, count++, i);
			*index = i;
			break;
		case TokenString:
			pushPropertyValue(property, count++, i);
			*index = i;
			break;
		case TokenCloseTuple:
			*index = i;
			return true;
		case TokenSpace:
			break;
		default:
			return false;
		}
	}
	return true;
}

int StageWorldFileParser::pushMacro(const char *macroname, const char *entityname,
								   int line, int startitem)
{	
	int macroCounter = _macros->size();
	StageMacro macro;

	macro.macroname = macroname;
	macro.entityname = entityname;
	macro.line = line;
	macro.startitem = startitem;
	_macros->push_back(macro);

	LOG<<"Pushed Macro with macroname = "<<macro.macroname<<", and entityname = "<<macro.entityname<<" in position "<<macroCounter+1;
	LOG<<"Macro vector is now size "<<_macros->size();

	return macroCounter;
}



///////////////////////////////////////////////////////////////////////////
// reset the entity list
void StageWorldFileParser::resetEntities()
{
	LOG<<"In resetEntities"<<endl;
	_entities->clear();
}


///////////////////////////////////////////////////////////////////////////
// push a entity
int StageWorldFileParser::pushEntity(int parent, const char *type)
{
	StageEntity entity;
	entity.parent = parent;
	entity.type = type;

	int entityCount = _entities->size();;
	_entities->push_back(entity);
		
	return entityCount;
}


///////////////////////////////////////////////////////////////////////////
// Get the number of _entities
int StageWorldFileParser::numEntities()
{
	return _entities->size();
}

///////////////////////////////////////////////////////////////////////////
// Get a entity (returns the entity type value)
const char *StageWorldFileParser::entityType(int entity)
{
	if (entity < 0 || entity >= _entities->size())
		return 0;
	return _entities->get(entity).type;
}


///////////////////////////////////////////////////////////////////////////
// Lookup a entity number by type name
// Returns -1 if there is no entity with this type
int StageWorldFileParser::findEntity(const char *type)
{
	for (int entity = 0; entity < numEntities(); entity++)
	{
		if (stricmp(entityType(entity), type) == 0)
			return entity;
	}
	return -1;
}


///////////////////////////////////////////////////////////////////////////
// reset the property list
void StageWorldFileParser::resetProperties()
{
	LOG<<"In resetProperties"<<endl;
	int i;
	StageProperty property;
	
	for (i = 0; i < _properties->size(); i++)
	{
		property = _properties->get(i);
		free(property.values);
	}
	_properties->clear();
}


///////////////////////////////////////////////////////////////////////////
// push an property
int StageWorldFileParser::pushProperty(int entity, const char *name, int line)
{
	int i;
	const StageProperty *property=0;
	
	for (i = 0; i < _properties->size(); i++)
	{
		property = _properties->getRef(i);
		if (property->entity != entity)
			continue;
		if (stricmp(property->name, name) == 0)
			return i;
	}
		
	StageProperty newProp ;

	newProp.entity = entity;
	newProp.name = name;
	newProp.value_count = 0;
	newProp.values = NULL;
	newProp.line = line;
	newProp.used = false;
	
	_properties->push_back(newProp);
	return i;
}


///////////////////////////////////////////////////////////////////////////
// push an property value
void StageWorldFileParser::pushPropertyValue(int property, int index, int value_item)
{
	if(property < 0)
		return;
	StageProperty prop;

	if(property <= _properties->size())
	{
		prop = _properties->get(property);
	}

	// Expand the array if it's too small
	if (index >= prop.value_count)
	{
		prop.value_count = index + 1;
		prop.values = (int*) realloc(prop.values, prop.value_count * sizeof(int));
	}
	
	// Set the relevant value
	prop.values[index] = value_item;

	_properties->put(property,prop);
}


///////////////////////////////////////////////////////////////////////////
// Get an property 
int StageWorldFileParser::getProperty(int entity, const char *name)
{
	//LOG<<"In getProperty("<<entity<<","<<name<<")"<<endl;
	// Find first instance of property
	for (int i = 0; i < _properties->size(); i++)
	{
		const StageProperty *property = _properties->getRef(i);
		if (property->entity != entity)
			continue;
		if (stricmp(property->name, name) == 0)
		{
			return i;
		}
	}

	return -1;
}




///////////////////////////////////////////////////////////////////////////
// Get the value of an property 
const char *StageWorldFileParser::getPropertyValue(int property, int index)
{
	if(property < 0)
		return 0;

	StageProperty prop = _properties->get(property);
		
	if( !(index < prop.value_count) )
		return 0;
	
	prop.used = true;

	_properties->put(property,prop);

	return getValue(prop.values[index]);
}



///////////////////////////////////////////////////////////////////////////
// Read a string
const char *StageWorldFileParser::getStr(int entity, const char *name, const char *value)
{
	int property = getProperty(entity, name);
	if (property < 0)
		return value;
	return getPropertyValue(property, 0);
}


///////////////////////////////////////////////////////////////////////////
// Read a float
double StageWorldFileParser::getDbl(int entity, const char *name, double value)
{
	int property = getProperty(entity, name);
	if (property < 0)
		return value;

	const char* prop = getPropertyValue(property, 0); 
	double retFloat = (prop == 0) ? value : atof(prop);

	LOG<<"ReadFloat string = "<<prop<<", and float = "<<retFloat;

	return retFloat;
}


///////////////////////////////////////////////////////////////////////////
// Read a float from a tuple
double StageWorldFileParser::getDbl(int entity, const char *name, int index, double value)
{
	LOG<<"In ReadTupleFloat("<<entity<<","<<name<<","<<index<<","<<value<<")"<<endl;
	int property = getProperty(entity, name);
	if (property < 0)
	{
		LOG<<"ReadTupleFloat is returning the error value "<<value<<endl;
		return value;
	}
	const char* propVal = getPropertyValue(property, index);
	if(propVal == 0)
		return value;

	double retval = atof(propVal);

	LOG<<"ReadTupleFloat is returning "<<retval<<endl;
	return retval;
}


