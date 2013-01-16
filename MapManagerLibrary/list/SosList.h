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

#ifndef SOSLIST_H
#define SOSLIST_H

#include "SosUtil.h"
#include "../logger/Logger.h"

#define MODE_QUEUE 1
#define MODE_STACK 2
#define MODE_ORDERED_ASC 3
#define MODE_ORDERED_DESC 4

//Interface for reading lists of a given template
template <class T>
class IListReader
{
	public:
		virtual void resetIterator() = 0;
		virtual bool readNext(T& retVal) = 0;
		virtual bool readVal(const T &val, T& retVal) = 0;
		virtual long getListSize() = 0;
};


template <class T>
struct ListNode
{
	T val;
	ListNode<T>* prev;	
	ListNode<T>* next;
};


template <class T>
class List : public IListReader<T>
{
public:
	List();
	~List();

	void setModeQueue();
	void setModeStack();
	void setModeOrderedAsc();
	void setModeOrderedDesc();

	void push(const T &val);
	bool popHead(T& val);
	bool popTail(T& val);

	bool pop();

	bool readHead(T& val);
	bool readTail(T& val);
	bool popVal(const T &val, T& retVal);
	bool readVal(const T &val, T& retVal);
	bool readValNum(long nodeNumber, T& retVal);

	bool erase(const T& val);

	long find(const T &val);
	void clear();
	
	void resetIterator();
	bool readNext(T& retVal);

	long getListSize();

	bool remove();//removes the node currently being accessed with the iterator

	bool replace(const T& val);
	bool replaceSequence(const T &firstVal, const T& lastVal, List<T>& sequenceReplacement);

protected:
	void deleteNode(ListNode<T>* node);

private:
	ListNode<T>* findRef(const T& val);

	ListNode<T>* _head;
	ListNode<T>* _tail;
	ListNode<T>* _current;
	ListNode<T>* _iterCurrent;
	bool _advanceIteration;

	short _mode;

	long _listSize;

	DEF_LOG
};


template <class T>
class ListUnordered : public IListReader<T>
{
public:
	ListUnordered();
	~ListUnordered();

	void setModeQueue();
	void setModeStack();

	void push(const T &val);
	void pushUnique(const T &val);
	bool popHead(T& val);
	bool popTail(T& val);

	bool pop();

	bool readHead(T& val);
	bool readTail(T& val);
	bool popVal(const T & val, T& retVal);
	bool readVal(const T & val, T& retVal);
	bool readValNum(long nodeNumber, T& retVal);

	long find(const T & val);
	void clear();

	bool erase(const T& val);
	
	void resetIterator();
	bool readNext(T& retVal);

	long getListSize();

	bool remove();//removes the node currently being accessed by the iterator

	bool replace(const T& val);
	bool replaceSequence(const T & firstVal, const T & lastVal, ListUnordered<T>& sequenceReplacement);

	
protected:
	void deleteNode(ListNode<T>* node);

private:
	ListNode<T>* findRef(const T & val);

	ListNode<T>* _head;
	ListNode<T>* _tail;
	ListNode<T>* _current;
	ListNode<T>* _iterCurrent;
	bool _advanceIteration;
	//ListNode<T>* _iterPrev;
	//bool _iterFinished;

	short _mode;

	long _listSize;

	

	DEF_LOG
};


template List<int>;
template List<double>;
template List<float>;
template List<long>;
template List<PathNode>;
template List<LineXY>;

#if 0  //enable-disable lists this way

template List<DrawableRectangle>;
template List<DrawableRectangleFilled>;
template List<DrawableLine>;
template List<RectObject>;
#endif

template List<LineXYLayer>;


//this is a list of LayerValue, each of which contains a list of points belonging to a layer
template List<LayerValue<List<PointXY>*> >;
template List<LayerValue<float> >;

#ifndef POINTXYZ_ALREADY_DEFINED
template List<PointXY>;
#endif

template List<PointXYZ>;


template ListUnordered<SosPose>;

template<class T>
List<T>::List()
{
	_head= _tail= _current= _iterCurrent = 0;
	_listSize = 0;
	_mode = MODE_QUEUE;
	//_iterFinished = false;

	//GET_FILE_LOG
	LOGGING_OFF;
}

template<class T>
List<T>::~List()
{
	LOG<<"In List destructor"<<endl;
	clear();
	LOG<<"At end of List destructor"<<endl;
}

template<class T>
void List<T>::clear()
{
	LOG<<"In List::clear"<<endl;
	ListNode<T> * oldHead = 0;
	ListNode<T> * current = 0;
	
	oldHead = _head;
	_head = 0;
	current = oldHead;
	
	
	_tail = _iterCurrent = _current = 0;
	_listSize = 0;

	
	LOG<<"About to go thru list and delete them all";
	while(current != 0 && oldHead != 0)
	{
		current = current->next;
		delete oldHead;
		oldHead = current;
	}	
	LOG<<"At end of List::clear"<<endl;
}

template<class T>
void List<T>::setModeOrderedAsc()
{
	ListNode<T> * oldHead = 0;
	ListNode<T> * current = 0;
	if(_mode != MODE_ORDERED_ASC)
	{
		_mode = MODE_ORDERED_ASC;
		oldHead = _head;
		current = _head;
		
		_head = 0;
		_tail = 0;
		_listSize = 0;
		
		while(current != 0)
		{
			current = current->next;
			push(oldHead->val);
			delete oldHead;
			oldHead = current;
		}
	}
}

template<class T>
void List<T>::setModeOrderedDesc()
{
	ListNode<T> * oldHead = 0;
	ListNode<T> * current = 0;
	if(_mode != MODE_ORDERED_DESC)
	{
		_mode = MODE_ORDERED_DESC;
		oldHead = _head;
		current = _head;
		
		_head = 0;
		_tail = 0;
		_listSize = 0;
		
		while(current != 0)
		{
			current = current->next;
			push(oldHead->val);
			delete oldHead;
			oldHead = current;
		}
	}
}

template<class T>
long List<T>::getListSize()
{
	return _listSize;
}

template<class T>
void List<T>::setModeQueue()
{
	_mode = MODE_QUEUE;
}

template<class T>
void List<T>::setModeStack()
{
	_mode = MODE_STACK;
}


template<class T>
void List<T>::push(const T & val)
{
	ListNode<T> *node = new ListNode<T>;
	node->val = val;

	//LOG<<"Copied val to node"<<endl;

	ListNode<T> *current = 0;
	ListNode<T> *prev = 0;
	ListNode<T> *next = 0;

	if(_current == 0)
		_current = _head;

	_listSize ++;

	if(_head == 0)
	{
		//LOG<<"List is empty - pointing head to current node"<<endl;
		_head = node;
		_tail = node;
		node->next = 0;
		node->prev = 0;
		_iterCurrent = _head;

		//LOG<<"List::Push() returning"<<endl;
		return;
	}

	//LOG<<"Did NOT point head to the current node"<<endl;

	switch(_mode)
	{
	case MODE_QUEUE:
		_tail->next = node;
		node->prev = _tail;
		node->next = 0;
		_tail = node;

	//	if(_iterCurrent == 0 && _iterFinished)
	//	{
	//		_iterCurrent = _tail;
			//LOG<<"1.changed _iterCurrent";
	//	}
		return;
		break;

	case MODE_STACK:
		node->next = _head;
		node->prev = 0;
		_head->prev = node;
		_head = node;
		return;
		break;
	}

	//current = _current;

	if(_mode == MODE_ORDERED_ASC)
	{		
		if(_current->val < val)
		{
			while(_current != 0 && _current->val < val)
			{
				_current = _current->next;
			}
			current = _current;
		}
		else if(_current->val > val)
		{
			while(_current != 0 && _current->val > val)
			{
				_current = _current->prev;
			}
			//the new node will be inserted BEFORE the current node,
			//so go one to the right
			current = (_current == 0) ? _head : _current->next;
			
		}		
	}else if(_mode == MODE_ORDERED_DESC)
	{
		if(_current->val < val)
		{
			while(_current != 0 && _current->val < val)
			{
				_current = _current->prev;
			}
			current = (_current == 0) ? _head : _current->next;

		}
		else if(_current->val > val)
		{
			while(_current != 0 && _current->val > val)
			{
				_current = _current->next;
			}

			current = _current;
		}

	}

	//if we're at the end of the list, insert at the tail
	if(current == 0)
	{
		_tail->next = node;
		node->prev = _tail;
		node->next = 0;
		_tail = node;

		//if(_iterCurrent == 0 && _iterFinished)
		//{
		//	_iterCurrent = _tail;
			//LOG<<"2.changed _iterCurrent";

		//}
		return;
	}
	if(current == _head)
	{
		_head->prev = node;
		node->next = _head;
		node->prev = 0;
		_head = node;
		return;
	}
	//the current node is NOT less/greater than the val being inserted
	//so insert before it
	prev = current->prev;
	next = current;//->next;
	
	if(prev != 0)
	{
		//LOG<<"prev = ("<<prev->x<<","<<prev->y<<") val="<<prev->value;
		prev->next = node;
	}

	node->prev = prev;
	
	if(next != 0)
		next->prev = node;

	node->next = next;

	return;	
}

template<class T>
bool List<T>::popHead(T& val)
{
	if(_head == 0)
		return false;

	if(_head == _tail)
		_tail = 0;

	_listSize--;

	val = _head->val;

	ListNode<T>* del = _head;
	_head = _head->next;

	if(_head != 0)
	{
		_head->prev = 0;
	}
	delete del;
	_current = _head;	

	return true;
}

template<class T>	
bool List<T>:: pop()
{
	T discard;
	return popHead(discard);
}

template<class T>	
bool List<T>::popTail(T& val)
{
	if(_tail == 0)
		return false;

	if(_tail == _head)
	{
		_head = 0;
	}

	_listSize--;

	val = _tail->val;
	ListNode<T>* del = _tail;
	_tail = _tail->prev;

	if(_tail != 0)
	{
		_tail->next = 0;
	}
	delete del;
	_current = 0;

	return true;

}
//---------------------------------------------
template<class T>
bool List<T>::readHead(T& val)
{
	if(_head == 0)
		return false;

	val = _head->val;
	
	return true;
}

template<class T>	
bool List<T>::readTail(T& val)
{
	if(_tail == 0)
		return false;
	
	val = _tail->val;
	
	return true;
}


//--------------------------------------------

template<class T>
bool List<T>::popVal(const T & val, T& retVal)
{
	ListNode<T>* node = findRef(val);

	if(node == 0)
	{
		return false;
	}
	retVal = node->val;	

	deleteNode(node);
	_listSize--;
	return true;
}

template<class T>
bool List<T>:: erase(const T& val)
{
	ListNode<T>* node = findRef(val);

	if(node == 0)
	{
		return false;
	}
	deleteNode(node);
	_listSize--;
	return true;

}

template<class T>
bool List<T>::readVal(const T & val, T& retVal)
{
	ListNode<T>* node = findRef(val);
	
	if(node == 0)
		return false;

	retVal = _current->val;
	return true;
}

template<class T>
bool List<T>::replace(const T& val)
{
	ListNode<T>* node = findRef(val);

	if(node == 0)
		return false;

	node->val = val;

	return true;
}

template<class T>
ListNode<T>* List<T>::findRef(const T & val)
{
	if(_head == 0)
	{
		return 0;
	}

	if(_current != 0 && _current->val == val)
	{
		return _current;
	}
	else
	{
		if(_mode == MODE_QUEUE || _mode == MODE_STACK)
		{
			_current = _head;
			while(_current != 0 && _current->val != val)
			{
				_current = _current->next;
			}
		}
		else if(_mode == MODE_ORDERED_ASC)
		{
			if(_current == 0) 
				_current = _head;

			//if the current node is greater than the one being looked for, search backwards
			if(_current->val > val)
			{
				while(_current != 0 && _current->val != val && _current->val > val)
				{
					_current = _current->prev;
				}
			}
			else //otherwise search forwards
			{
				while(_current != 0 && _current->val != val && _current->val < val)
				{
					_current = _current->next;
				}
			}			
		}
		else if(_mode == MODE_ORDERED_DESC)
		{
			if(_current == 0) 
				_current = _head;

			//if the current node is smaller than the one being looked for, search backwards
			if(_current->val < val)
			{
				while(_current != 0 && _current->val != val && _current->val < val)
				{
					_current = _current->prev;
				}
			}
			else //otherwise search forwards
			{
				while(_current != 0 && _current->val != val && _current->val > val)
				{
					_current = _current->next;
				}
			}			
		}

		//if not found, return false
		if(_current == 0 || _current->val != val)
		{
			return 0;
		}
		else
		{
			return _current;
		}
	}



}

template<class T>
bool List<T>::readValNum(long nodeNumber, T& retVal)
{
	ListNode<T>* node = 0;
	if(nodeNumber < 0 || nodeNumber > _listSize)
	{
	//	LOG<<"readValNum returning false because nodeNumber = "<<nodeNumber;
		return false;
	}

	long count = 0;

	_current = _head;
	while(_current != 0 && count<nodeNumber)
	{
		_current = _current->next;
		count++;
	}
	//if not found, return false
	if(_current == 0)
	{
		return false;
	}

	retVal = _current->val;
	return true;
}

template<class T>
long List<T>::find(const T & val)
{
	if(_head == 0)
		return -1;

	long counter = 0;
	_current = _head;

	while(_current != 0 && _current->val != val )
	{
		counter++;
		_current = _current->next;
	}

	if(_current == 0)
		return -1;

	return counter;
}

template<class T>	
void List<T>::resetIterator()
{
	_iterCurrent = _head;
	_advanceIteration = false;
	
}

template<class T>
bool List<T>::readNext(T& retVal)
{
	if(_iterCurrent == 0 || _head == 0)
	{
		return false;
	}
	if(!_advanceIteration)
	{
		_advanceIteration = true;
	}
	else
	{
		_iterCurrent = _iterCurrent->next;
	}
	
	if(_iterCurrent == 0)
		return false;

	retVal = _iterCurrent->val;
	//_iterCurrent = _iterCurrent->next;
		
	return true;
}

template<class T>
bool List<T>::remove()
{
	if(_iterCurrent == 0)
		return false;

	ListNode<T>* prev = _iterCurrent;//->prev;
	_iterCurrent = _iterCurrent->next;
	_advanceIteration = false;
	
	deleteNode(prev);
	_listSize--;
	return true;
}


template<class T>
void List<T>::deleteNode(ListNode<T>* node)
{
	if(node == _head && _head != 0)
	{
		_head = _head->next;
		if(_head != 0)
		{
			_head->prev = 0;
		}
		if(_current == node)
			_current = _head;

		delete node;
		return;
	}
	if(node == _tail && _tail != 0)
	{
		_tail = _tail->prev;
		if(_tail != 0)
		{
			_tail->next = 0;
		}
		if(_current == node)
			_current = _head;

		delete node;
		return;
	}
	ListNode<T>* prev = node->prev;
	ListNode<T>* next = node->next;

	if(prev != 0)
	{
		prev->next = next;
	}
	if(next != 0)
	{
		next->prev = prev;
	}

	_current = 0;
	//if(_current == node)
	//	_current = _head;

	delete node;
	//_current = _head;
}

//this method replaces a sequence of nodes in the list, starting with firstVal and ending with lastVal,
//with another list.  It only works in Queue or Stack modes
template<class T>
bool List<T>:: replaceSequence(const T & firstVal, const T & lastVal, List<T>& sequenceReplacement)
{
	if(_mode != MODE_QUEUE && _mode != MODE_STACK || _head == 0)
		return false;

	ListNode<T>* firstNode = 0;
	ListNode<T>* lastNode = 0;
	ListNode<T>* beforeFirstNode = 0;
	ListNode<T>* afterLastNode = 0;

	_current = _head;

	//make sure that the firstVal comes before lastVal, otherwise return false
	while(_current != 0 && _current->val != firstVal)
	{
		_current = _current->next;
	}

	if(_current == 0)
		return false;
	
	firstNode = _current;
	beforeFirstNode = firstNode->prev;

	while(_current != 0 && _current->val != firstVal)
	{
		_current = _current->next;
	}

	if(_current == 0)
		return false;
	
	lastNode = _current;
	afterLastNode = lastNode->next;

	_current = firstNode->next;

	//delete the desired sequence
	while(firstNode != lastNode)
	{
		deleteNode(firstNode);
		firstNode = _current;
		_current = _current->next;
	}


	//now insert the new sequence into this list
	sequenceReplacement.resetIterator();

	T val;
	_current = 0;

	while(sequenceReplacement.readNext(val))
	{
		_current = new ListNode<T>;
		_current->val = val;
		_current->prev = beforeFirstNode;
		beforeFirstNode->next = _current;
		beforeFirstNode = _current;
	}

	if(_current != 0)//if we inserted at least one item
	{
		_current->next = afterLastNode;
		afterLastNode->prev = _current;
	}


	return true;

}

//===========================================================================

template<class T>
ListUnordered<T>::ListUnordered()
{
	_head= _tail= _current= _iterCurrent = 0;
	_listSize = 0;
	_mode = MODE_QUEUE;
//	GET_FILE_LOG
	LOGGING_OFF;
}

template<class T>
ListUnordered<T>::~ListUnordered()
{
	clear();	
}

template<class T>
void ListUnordered<T>::clear()
{
	LOG<<"In ListUnordered<T>::clear()"<<endl;
	ListNode<T> * oldHead = 0;
	ListNode<T> * current = 0;
	
	oldHead = _head;
	current = _head;
	
	_head = _iterCurrent = _tail = 0;
	_current = 0;
	_listSize = 0;
	//_iterFinished = false;
	
	while(current != 0)
	{
		current = current->next;
		delete oldHead;
		oldHead = current;
	}	
	LOG<<"At end of ListUnordered<T>::clear()"<<endl;
}

template<class T>
long ListUnordered<T>::getListSize()
{
	return _listSize;
}

template<class T>
void ListUnordered<T>::setModeQueue()
{
	_mode = MODE_QUEUE;
}

template<class T>
void ListUnordered<T>::setModeStack()
{
	_mode = MODE_STACK;
}


template<class T>
void ListUnordered<T>::push(const T & val)
{
	LOG<<"ListUnordered<T>::push(): About to create a new Node"<<endl;
	ListNode<T> *node = new ListNode<T>;
	LOG<<"ListUnordered<T>::push(): Finished creating a new node"<<endl;

	node->val = val;

	LOG<<"ListUnordered<T>::push() Copied val to node"<<endl;

	if(_current == 0)
		_current = _head;

	_listSize ++;

	if(_head == 0)
	{
		LOG<<"ListUnordered is empty - pointing head to current node"<<endl;
		_head = node;
		_tail = node;
		node->next = 0;
		node->prev = 0;
		_iterCurrent = _head;

		LOG<<"ListUnordered::Push() returning"<<endl;
		return;
	}

	LOG<<"Did NOT point head to the current node"<<endl;

	switch(_mode)
	{
	case MODE_QUEUE:
		_tail->next = node;
		node->prev = _tail;
		node->next = 0;
		_tail = node;

		return;
		break;

	case MODE_STACK:
		node->next = _head;
		node->prev = 0;
		_head->prev = node;
		_head = node;
		return;
		break;
	default:
		return;//this is an error, shouldn't happen
	}
}

template<class T>
void ListUnordered<T>::pushUnique(const T &val)
{
	_current = _head;
	ListNode<T>* prev = 0;

	while(_current != 0)
	{
		prev = _current;
		_current = _current->next;
		if(prev->val == val)
		{
			deleteNode(prev);
		}
	}

	_current = 0;
	push(val);
}

template<class T>
bool ListUnordered<T>::popHead(T& val)
{
	if(_head == 0)
		return false;

	if(_head == _tail)
		_tail = 0;

	_listSize--;

	val = _head->val;

	ListNode<T>* del = _head;
	_head = _head->next;

	if(_head != 0)
	{
		_head->prev = 0;
	}
	delete del;
	_current = _head;	

	return true;
}

template<class T>	
bool ListUnordered<T>:: pop()
{
	T discard;
	return popHead(discard);
}

template<class T>	
bool ListUnordered<T>::popTail(T& val)
{
	if(_tail == 0)
		return false;

	if(_tail == _head)
	{
		_head = 0;
	}

	_listSize--;

	val = _tail->val;
	ListNode<T>* del = _tail;
	_tail = _tail->prev;

	if(_tail != 0)
	{
		_tail->next = 0;
	}
	delete del;
	_current = 0;

	return true;

}
//---------------------------------------------
template<class T>
bool ListUnordered<T>::readHead(T& val)
{
	if(_head == 0)
		return false;

	val = _head->val;
	
	return true;
}

template<class T>	
bool ListUnordered<T>::readTail(T& val)
{
	if(_tail == 0)
		return false;
	
	val = _tail->val;
	
	return true;
}


//--------------------------------------------

template<class T>
bool ListUnordered<T>::popVal(const T & val, T& retVal)
{
	ListNode<T>* node = findRef(val);

	if(node == 0)
	{
		return false;
	}
	retVal = node->val;	

	deleteNode(node);
	_listSize--;
	return true;
}

template<class T>
bool ListUnordered<T>:: erase(const T& val)
{
	ListNode<T>* node = findRef(val);

	if(node == 0)
	{
		return false;
	}
	deleteNode(node);
	_listSize--;
	return true;

}

template<class T>
bool ListUnordered<T>::readVal(const T & val, T& retVal)
{
	ListNode<T>* node = findRef(val);
	
	if(node == 0)
		return false;

	retVal = _current->val;
	return true;
}


template<class T>
bool ListUnordered<T>::replace(const T& val)
{
	ListNode<T>* node = findRef(val);

	if(node == 0)
		return false;

	node->val = val;

	return true;
}

template<class T>
ListNode<T>* ListUnordered<T>::findRef(const T & val)
{
	if(_head == 0)
	{
		return 0;
	}

	if(_current != 0 && 
		_current->val == val)
	{
		return _current;
	}
	else
	{
		_current = _head;
		while(_current != 0 && 
			_current->val != val)
		{
			_current = _current->next;
		}
		
		//if not found, return false
		if(_current == 0 || _current->val != val)
		{
			return 0;
		}
		else
		{
			return _current;
		}
	}
}

template<class T>
bool ListUnordered<T>::readValNum(long nodeNumber, T& retVal)
{
	ListNode<T>* node = 0;
	if(nodeNumber < 0 || nodeNumber > _listSize)
	{
	//	LOG<<"readValNum returning false because nodeNumber = "<<nodeNumber;
		return false;
	}

	long count = 0;

	_current = _head;
	while(_current != 0 && count<nodeNumber)
	{
		_current = _current->next;
		count++;
	}
	//if not found, return false
	if(_current == 0)
	{
		return false;
	}

	retVal = _current->val;
	return true;
}

template<class T>
long ListUnordered<T>::find(const T & val)
{
	if(_head == 0)
		return -1;

	long counter = 0;
	_current = _head;

	while(_current != 0 && _current->val != val )
	{
		counter++;
		_current = _current->next;
	}

	if(_current == 0)
		return -1;

	return counter;
}

template<class T>	
void ListUnordered<T>::resetIterator()
{
	_iterCurrent = _head;
	_advanceIteration = false;
	LOG<<"ListUnordered<T>::resetIterator set _iterCurrent = "<<_iterCurrent;
}

template<class T>
bool ListUnordered<T>::readNext(T& retVal)
{
	LOG<<"readNext() : _iterCurrent = "<<_iterCurrent<<" and _advanceIteration = "<<_advanceIteration;
	if(_iterCurrent == 0 || _head == 0)
	{
		return false;
	}
	if(!_advanceIteration)
	{
		_advanceIteration = true;
	}
	else
	{
		_iterCurrent = _iterCurrent->next;
	}

	if(_iterCurrent == 0)
		return false;
	
	retVal = _iterCurrent->val;
	//_iterCurrent = _iterCurrent->next;

	//if(_iterCurrent == 0)
	//	_iterFinished = true;
	
	return true;
}

template<class T>
bool ListUnordered<T>::remove()
{
	if(_iterCurrent == 0)
		return false;

	ListNode<T>* prev = _iterCurrent;//->prev;
	_iterCurrent = _iterCurrent->next;
	_advanceIteration = false;
	
	deleteNode(prev);
	_listSize--;
	return true;
}

template<class T>
void ListUnordered<T>::deleteNode(ListNode<T>* node)
{
	if(node == _head && _head != 0)
	{
		_head = _head->next;
		if(_head != 0)
		{
			_head->prev = 0;
		}
		if(_current == node)
			_current = _head;

		delete node;
		return;
	}
	if(node == _tail && _tail != 0)
	{
		_tail = _tail->prev;
		if(_tail != 0)
		{
			_tail->next = 0;
		}
		if(_current == node)
			_current = _head;

		delete node;
		return;
	}
	ListNode<T>* prev = node->prev;
	ListNode<T>* next = node->next;

	if(prev != 0)
	{
		prev->next = next;
	}
	if(next != 0)
	{
		next->prev = prev;
	}

//	if(_current == node)
//		_current = _head;

	delete node;
	//_current = _head;
	_current = 0;
	_iterCurrent = 0;//cancel the iterating through the list if it's running
}

//this method replaces a sequence of nodes in the list, starting with firstVal and ending with lastVal,
//with another list.  It only works in Queue or Stack modes
template<class T>
bool ListUnordered<T>::replaceSequence(const T & firstVal, const T & lastVal, ListUnordered<T>& sequenceReplacement)
{
	ListNode<T>* firstNode = 0;
	ListNode<T>* lastNode = 0;
	ListNode<T>* beforeFirstNode = 0;
	ListNode<T>* afterLastNode = 0;

	_current = _head;

	//make sure that the firstVal comes before lastVal, otherwise return false
	while(_current != 0 && _current->val != firstVal)
	{
		_current = _current->next;
	}

	if(_current == 0)
		return false;
	
	firstNode = _current;
	beforeFirstNode = firstNode->prev;

	while(_current != 0 && _current->val != firstVal)
	{
		_current = _current->next;
	}

	if(_current == 0)
		return false;
	
	lastNode = _current;
	afterLastNode = lastNode->next;

	_current = firstNode->next;

	//delete the desired sequence
	while(firstNode != lastNode)
	{
		deleteNode(firstNode);
		firstNode = _current;
		_current = _current->next;
	}


	//now insert the new sequence into this list
	sequenceReplacement.resetIterator();

	T val;
	_current = 0;

	while(sequenceReplacement.readNext(val))
	{
		_current = new ListNode<T>;
		_current->val = val;
		_current->prev = beforeFirstNode;
		beforeFirstNode->next = _current;
		beforeFirstNode = _current;
	}

	if(_current != 0)//if we inserted at least one item
	{
		_current->next = afterLastNode;
		afterLastNode->prev = _current;
	}


	return true;

}

#endif