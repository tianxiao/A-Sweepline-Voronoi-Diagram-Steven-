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

#ifndef SOS_VECTOR_H
#define SOS_VECTOR_H

#include "../logger/Logger.h"

template <class T>
class VectorNode
{
public:
	VectorNode(long initSize, long globPos, const T& initVal);
	~VectorNode();
	T get(long position)
	{
	//	LOG<<"VectorNode::get("<<position<<"), size = "<<size;
		return _data[position];
	}
	void put(long position, const T& val)
	{
	//	LOG<<"VectorNode::put("<<position<<"), size = "<<size;
		_data[position]=val;
	}

	void erase(long position);
	int getSize() {return size;}

	const T* getRef(long pos){return _data + pos;}
	
	long getGlobalPos(){return pos;}
	void setGlobalPos(long globPos){pos = globPos;}
	VectorNode<T> *prev, *next;

private:
	T* _data;
	
	long pos;
	int size;

	DEF_LOG
};

template <class T>
class Vector
{
public:
	Vector(const T& initVal, long chunkSize = 100, bool loggingOn = false);
	~Vector();

	T get(long pos);
	void put(long pos, const T& val);
	void clear();
	long size() 
	{
		return _lastEntry +1;
	}
	long push_back(const T& val) 
	{
		put(_lastEntry + 1,val);
		return _lastEntry;
	}
	void erase(long pos);

	const T* getRef(long pos);
	DEF_LOG

private:
	VectorNode<T>* _data , *_current, *_last;
	long _maxSize;
	long _chunkSize;

	long _lastEntry;

	T _defaultVal;

	

};

//-------------------------------------------------

template <class T>
Vector<T>::Vector(const T& initVal, long chunkSize, bool loggingOn)
{
	
	//GET_FILE_LOG
	LOGGING_OFF
	

	LOG<<"In vector constructor "<<this;
	if(chunkSize < 1)
		chunkSize = 100;

	_chunkSize = chunkSize;
	_maxSize = _chunkSize -1;

	_defaultVal = initVal;
	_data = new VectorNode<T>(_chunkSize,0,_defaultVal);	
	_data->next = _data->prev = 0;
	_last = _current = _data;	

	_lastEntry = -1;
}

template <class T>
void Vector<T>::put(long pos, const T& val)
{
	if(pos < 0)
		return;
	LOG<<"Vector::put("<<pos<<"), _maxSize = "<<_maxSize<<", _lastEntry = "<<_lastEntry;
	VectorNode<T> * newNode = 0;
	while(pos > _maxSize)
	{
		LOG<<"Pos ("<<pos<<") > _maxSize("<<_maxSize<<") so increasing Vector size by"<<_chunkSize;
		newNode = new VectorNode<T>(_chunkSize,_last->getGlobalPos() + _last->getSize(),_defaultVal);
		
		_last->next = newNode;
		newNode->prev = _last;
		newNode->next = 0;
		_last = newNode;

		_maxSize += _chunkSize;
		_current = _last;
		LOG<<"Put("<<pos<<") created a new node with glob pos "<<newNode->getGlobalPos();
	}

	if(_current == 0)
		_current = _data;

	while(pos > _current->getGlobalPos() + _current->getSize() -1)
	{
		LOG<<"Going forward one because pos("<<pos<<") > _current->getGlobalPos()("<< _current->getGlobalPos()<<")+_chunkSize("<<_chunkSize<<")-1 = "<<_current->getGlobalPos() + _chunkSize -1;
		_current = _current->next;
	}

	while(pos < _current->getGlobalPos())
	{
		_current = _current->prev;
	}

	LOGCODE if(_current == 0)
	LOGCODE {
				LOG<<"Error: _current is 0, so didn't create a big enough vector for position"<<pos;
	LOGCODE }

	LOGCODE if(pos - _current->getGlobalPos() >= _chunkSize || pos < 0)
	LOGCODE {
		LOG<<"Error!!! Vector trying to put object into pos "<<pos - _current->getGlobalPos()<<" of Node with size "<<_current->getSize();
	LOGCODE }

	LOG<<"Pos = "<<pos<<", _current->getGlobalPos() = "<<_current->getGlobalPos() <<", so putting into pos "<<pos - _current->getGlobalPos();
	_current->put(pos - _current->getGlobalPos(), val);

	if(pos > _lastEntry)
		_lastEntry = pos;

}

template <class T>
Vector<T>::~Vector()
{
	clear();
	delete _data;
}

template <class T>
void Vector<T>::clear()
{
	if(_lastEntry < 0)
		return;

	_current = _data;
	while(_current != 0)
	{
		_current = _current->next;
		delete _data;
		_data = _current;
	}

	_data = new VectorNode<T>(_chunkSize,0,_defaultVal);
	//LOG<<"Clear() created a new _data node with glob pos "<<_data->getGlobalPos();
	_current = _last = _data;
	_data->prev = _data->next = 0;
	_lastEntry = -1;
	_maxSize = _chunkSize - 1;
}

template <class T>
const T* Vector<T>::getRef(long pos)
{
	if(pos < 0 || pos > _maxSize || pos > _lastEntry)
	{
		return 0;
	}

	if(_current == 0)
	{
		_current = _data;
	}

	if(pos > _current->getGlobalPos() + _current->getSize() -1)
	{
		_current = _current->next;
		while(_current != 0 && pos > _current->getGlobalPos() + _current->getSize()-1)
		{
			_current = _current->next;
		}
	}
	else if(pos < _current->getGlobalPos())
	{
		_current = _current->prev;
		while(_current != 0 && pos < _current->getGlobalPos())
		{
			_current = _current->prev;
		}
	}
		
	if(_current != 0)
	{
		return _current->getRef(pos - _current->getGlobalPos());
	}
	else
	{		
		return 0;
	}


}

template <class T>
T Vector<T>::get(long pos)
{
	LOG<<"In Vector::get("<<pos<<")";
	if(pos < 0 || pos > _maxSize)
	{
		LOG<<"Returning the default value because _maxSize ("<<_maxSize<<" < pos ("<<pos<<")"<<endl;
		return _defaultVal;
	}

	if(_current == 0)
	{
		_current = _data;
	}

	if(pos > _current->getGlobalPos() + _current->getSize() -1)
	{
		LOG<<"get("<<pos<<"): Pos > _current globalPos + size of "<<_current->getGlobalPos()<<" + "<<_current->getSize()<<endl;
		_current = _current->next;
		while(_current != 0 && pos > _current->getGlobalPos() + _current->getSize()-1)
		{
			_current = _current->next;
		}
	}
	else if(pos < _current->getGlobalPos())
	{
		LOG<<"get("<<pos<<"): Pos < _current globalPos of "<<_current->getGlobalPos()<<endl;
		_current = _current->prev;
		while(_current != 0 && pos < _current->getGlobalPos())
		{
			_current = _current->prev;
		}
	}
		
	if(_current != 0)
	{
		LOG<<"Vector::get("<<pos<<") returning value "<<pos - _current->getGlobalPos()<<" from node with glob pos "<<_current->getGlobalPos()<<" and chunk size "<<_chunkSize;;
		return _current->get(pos - _current->getGlobalPos());
	}
	else
	{
		LOG<<"Returning the default value because _current is null";
		return _defaultVal;
	}

}

template <class T>
void Vector<T>::erase(long pos)
{
	LOG<<"In "<<this<<" Vector::erase("<<pos<<")";
	if(pos < 0 || pos > _maxSize)
		return;

	if(_current == 0)
		_current = _data;

	if(pos > _current->getGlobalPos() + _current->getSize() -1)
	{
		LOG<<"Pos > _current globalPos + size of "<<_current->getGlobalPos()<<" + "<<_current->getSize();
		_current = _current->next;
		while(_current != 0 && pos > _current->getGlobalPos() + _current->getSize() -1)
		{
			_current = _current->next;
		}
	}
	else if(pos < _current->getGlobalPos())
	{
		LOG<<"Pos < _current globalPos of "<<_current->getGlobalPos();
		_current = _current->prev;
		while(_current != 0 && pos < _current->getGlobalPos())
		{
			_current = _current->prev;
		}
		
	}

	if(_current == 0)
	{
		LOG<<"_current == 0, returning without erasing anything";
		return;
	}

	LOG<<"Going to erase pos "<<pos - _current->getGlobalPos()<<" (from "<<pos<<" - "<<_current->getGlobalPos()<<") from node with gobal pos "<<_current->getGlobalPos()<<", size="<<_current->getSize();
	_current->erase(pos - _current->getGlobalPos());
	LOG<<"After erase, node with glob pos "<<_current->getGlobalPos()<<" has size = "<<_current->getSize();
	LOGCODE if(_current->next != 0)
	LOGCODE LOG<<"The node after _current has a glob pos of "<<_current->next->getGlobalPos();

	VectorNode<T> *tempNode = 0;
	if(_current->getSize() < 1)
	{
		LOG<<"_current after erase operation has size = 0, so deleting it";
		tempNode = _current;

		if(_current->prev != 0)
		{
			_current->prev->next = _current->next;
			LOG<<"Set _current->prev's next (addr = "<<_current->prev<<") to _current->next (addr = "<<_current->next;
		}

		if(_current->next != 0)
		{
			_current->next->prev = _current->prev;
			LOG<<"Set _current->next's prev (addr = "<<_current->next<<") to _current->prev (addr = "<<_current->prev;
		}

		if(tempNode == _data)
		{
			LOG<<"node being deleted is the first one, _data, so set _data to "<<tempNode->next<<endl;
			_data = tempNode->next;			
		}
		_current = _current->next;

		if(_last == tempNode)
		{
			LOG<<"_last = "<<_last<<endl;
			if(_last != 0)LOG<<"Incrementing _last to "<<_last->prev<<endl;
			_last = _last->prev;
		}

		LOG<<"About to delete tempNode, of address "<<tempNode<<endl;
		delete tempNode;
		tempNode = 0;
		LOG<<"Finished deleting tempNode"<<endl;

		if(_data == 0)
		{
			LOG<<"About to create a new VectorNode and make _data point to it"<<endl;
			_data = new VectorNode<T>(_chunkSize,0,_defaultVal);	
			LOG<<"created a new _data node"<<endl;
			_current = 0;
		}

		LOG<<"new _data's glob pos is "<<_data->getGlobalPos()<<endl;
		

		if(_last == 0)
			_last = _data;

	}
	else
	{
		_current = _current->next;
	}
	
	while(_current != 0)
	{
		LOG<<"subtracting 1 from node with global pos "<<_current->getGlobalPos();
		_current->setGlobalPos(_current->getGlobalPos() -1);

		LOGCODE if(_current->getGlobalPos() < 0)
		LOGCODE {
		LOGCODE 	LOG<<"Error: set global pos to "<<_current->getGlobalPos();
		LOGCODE }

		_current = _current->next;
	}

	_lastEntry--;
	_maxSize--;

}

//-------------------------------------------------
template <class T>
VectorNode<T>::VectorNode(long initSize, long globPos, const T& initVal)
{
//	GET_FILE_LOG
	LOGGING_OFF
	LOG<<"In VectorNode("<<initSize<<","<<globPos<<")";
	
	//LOGGING_OFF
	_data = new T[initSize];
	pos = globPos;

	for(int i = 0; i< initSize; i++)
		_data[i] = initVal;

	size = initSize;
	next = prev = 0;

}

template <class T>
VectorNode<T>::~VectorNode()
{
	delete[] _data;
}

template <class T>
void VectorNode<T>::erase(long position)
{
	int i = 0;
	if(position < 0)
	{
		LOG<<"Error!! Request to erase position "<<position;
		return;
	}

	for(i = position; i< size -1; i++)
	{

		_data[i] = _data[i + 1];
	}

	size --;
}

#endif