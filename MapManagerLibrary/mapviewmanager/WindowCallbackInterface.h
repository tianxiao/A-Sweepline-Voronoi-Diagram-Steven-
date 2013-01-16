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

#ifndef WINDOW_CALLBACK_INTERFACE_H
#define WINDOW_CALLBACK_INTERFACE_H
#include "../drawableobjects/DrawableObject.h"
#include "../commondefs/commonDefs.h"
#include "../mapmanagerextended/IJobProgressMonitor.h"

class WindowCallbackInterface
{
public:
	virtual void updateToolbarButtons() = 0;
	virtual void displayMessage(char* message) = 0;

	//requests the window handler to pop up a message on the screen
	//asking for an OK or Cancel to be clicked.
	//Expected to return TRUE if Ok is clicked, otherwise FALSE
	virtual bool popupOkCancel(char* title,char* message){return false;};

	//requests the window handler to pop up a message on the screen
	//with just an OK message.  No return value.
	virtual void popupOk(char* title,char* message){};

	virtual bool getStageOptions(StageWorldOptions& opt)
	{
		opt.hasLaser			= true;
		opt.hasSonar			= false;
		opt.initialPort			= 6665;
		
		return true;
	}

	//requests the window handler to pop up a dialog on the screen 
	//to get the options for generating a random map.  
	//This is just the default implementation, and should be overridden
	//by the window handler
	virtual bool getRandomMapOptions(WorldOptions & opt)
	{
		opt.avgRoomArea = 30;
		opt.createObstacles = true;
		opt.numRooms = 100;
		opt.percentObstacleArea = 20;
		opt.useBox = true;
		opt.useChair = opt.useDiningArea= false;
		opt.useRoomGridLayout = true;
		return true;
	}

	virtual bool loadImage(char* filePath)
	{
		return false;
	}

	virtual bool getNumber1(char* message, bool isInteger, float initNum, float min, float max,float& numRet)
	{
		return false;
	}

	virtual bool getSaveFileName(char* filepath, const char* fileTypeDecription, const char* defaultExtension)
	{
		return false;
	}

	virtual IJobProgressMonitor* getJobProgressMonitor()
	{
		return 0;
	}
};

class IScreenPainter
{
public:
	virtual void paintObject(DrawableObject*) = 0;
};

class MapViewManagerCallbackInterface
{
public:
	virtual bool getMapCoords(long viewX, long viewY, long& mapX, long& mapY)=0;
	virtual bool gridToScreen(float x, float y, long& screenX, long& screenY) = 0;

};


#endif