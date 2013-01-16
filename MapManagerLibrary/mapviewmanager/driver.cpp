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

#include <iostream.h>
#include "MapViewManager.h"

int testGenerateDelaunay(MapManager* mgr);

int main()
{
	MapViewManager* mgr = new MapViewManager();

	testGenerateDelaunay(mgr);

	cout<<"\nAbout to clear the Delaunay"<<endl;

	mgr->clearDelaunay();

	cout<<"\nAbout to delete mgr"<<endl;

	delete mgr;
	mgr = 0;

	cout<<"After deleting the mgr, finishing"<<endl;

	return 0;
}

int testGenerateDelaunay(MapViewManager* mgr)
{
	
	mgr->loadMapViewerMap("c:\\test.mvm");

	mgr->generateDelaunay(0.5f,1.0f,3.0f);


	return 0;
}