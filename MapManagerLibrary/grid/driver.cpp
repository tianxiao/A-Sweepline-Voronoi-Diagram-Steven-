#include <iostream.h>
#include "Grid3D.h"
#include "GridMap.h"

void DO_COPY_ROW(Grid3D<int>* g, int*);
void DO_NORMAL_METHOD(Grid3D<int>* g,int*);

#define NUMVALS 10

int main()
{
	GridMap<int> g(100,0,-1);

	//int *arr= new int[NUMVALS+6];
	//int *arr2 = new int[NUMVALS+6];
	//int *arr3 = new int[NUMVALS+6];

	int arr[NUMVALS+6];
	int arr2[NUMVALS+6];
	int arr3[NUMVALS+6];

	int i = 0;
	for(i = 0; i< NUMVALS+6; i++)
	{
		arr[i] =  i+1;
		arr2[i] = arr3[i] = -1;
		
		g.updateGridRef(arr[i],i,1);
	}

	GridMap<int> g2(100,0,-1);
	g2.copy(&g);
	double corr = g2.correlateMap(&g);
	cout<<"\ncorrelation = "<<corr<<endl;
	g2.updateGridRef(1929,10,20);
	cout<<"\ncorrelation = "<<g2.correlateMap(&g)<<endl;

	for(int x = 0; x < 5; x++)
	{
		cout<<endl;
		for(int y = 0; y<5; y++)
		{
			cout<<" ["<<x<<","<<y<<"] = "<<g.getGridRef(x,y)<<" and "<<g2.getGridRef(x,y);

		}
	}

	
	
//	cout<<"Grid3D dimensions are east = "<<g.getDimensions(EAST)<<endl;

	DO_COPY_ROW(&g,arr2);
	DO_NORMAL_METHOD(&g,arr3);
/*
	for(i = 0; i< NUMVALS+6; i++)
	{
		cout<<arr[i]<<" \t"<<arr2[i]<<"\t"<<arr3[i]<<endl;
		if(arr3[i] != arr2[i] )
		{
			cout<<"Error at ["<<i<<"]: arr = "<<arr[i]<<", arr2 = "<<arr2[i]<<", arr3 = "<<arr3[i];
			break;
		}
	}

	cout<<endl;*/
//	cout<<"Right before trying to delete the arrays"<<endl<<endl;
	//delete[] arr;
	//delete[] arr2;
	//delete[] arr3;
//	arr = 0;
//	arr2 = 0;
//	arr3 = 0;
//	cout<<"Right before the end..."<<endl<<endl;

	return 0;
}

void DO_COPY_ROW(Grid3D<int>* g,int * arr2)
{
	bool retval = g->copyRow(arr2,1,-5,NUMVALS-1);
	if(!retval)
	{
		cout<<"Error! grid map returned false"<<endl;
	}
	
}

void DO_NORMAL_METHOD(Grid3D<int>* g,int * arr3)
{
	int val = 0;
	for(int i = -5; i< NUMVALS; i++)
	{
		arr3[i+5] = g->getGridRef(i,1);
	}
}
