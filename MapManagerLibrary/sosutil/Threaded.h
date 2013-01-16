#ifndef THREAD_HEADER_H
#define THREAD_HEADER_H

#include <windows.h>

class Threaded
{
public:
	friend DWORD WINAPI runThread(void *arg);
	Threaded()
	{
		_myThread = 0;
		_isRunning = false;
		_threadFinished = true;
	}

	virtual ~Threaded()
	{
		if(_myThread != 0)
		{
			stop();			
		}
	}

	bool start()
	{
		DWORD ret = 0;

		if(_myThread != 0)
		{
			return false;
		}
		_myThread = CreateThread(0, 0, &runThread, (void*)(this), 0, &ret);
		_isRunning = !(_myThread == 0);
		_threadFinished = !_isRunning;

		return _isRunning;
	}

	bool stop()
	{
		if(_myThread != 0)
		{
			//the running thread should check this in its while/for loop
			//and stop when it goes false
			_isRunning = false;

			while(!_threadFinished) 
			{
				Sleep(1);//sleep for a millisecond
			}
		//	TerminateThread(_myThread, 1);
			_myThread = 0;
		}
		else
		{
			return false;
		}
		return true;
	}

	bool isRunning()
	{
		return _isRunning;
	}

	//this method should be overridden, now it does nothing
	virtual void run()
	{

	}

protected:
	//the thread should call this just before it finishes
	void threadFinished()
	{
		_threadFinished = true;
	}

	bool _isRunning;

private:
	bool _threadFinished;
	HANDLE _myThread;

};



#endif