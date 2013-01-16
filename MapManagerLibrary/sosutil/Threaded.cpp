#include "Threaded.h"



DWORD WINAPI runThread(void *arg)
{
	if(arg == 0)
		return 0;

	//cout<<"In runThread"<<endl;

	Threaded* thread = (Threaded*)arg;

	thread->run();

	//cout<<"After calling thread->run"<<endl;

	thread->_myThread = 0;

	return 1;
}