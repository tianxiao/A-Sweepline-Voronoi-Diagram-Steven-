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

#ifndef CARMEN_TRANSLATOR_H
#define CARMEN_TRANSLATOR_H

#include "GridMap.h"
#include "SosUtil.h"
#include <iostream.h>
#include <io.h>
#include "../logger/Logger.h"


#define VERSION "v020"
#define LABEL "CARMENMAPFILE"


#define NO_RECORD          (0 << 0)
#define GRIDMAP_RECORD     (1 << 0)
#define OFFLIMITS_RECORD   (1 << 1)
#define PLACES_RECORD      (1 << 2)
#define EXPECTED_RECORD    (1 << 3)
#define LASERSCANS_RECORD  (1 << 4)
#define CREATOR_RECORD     (1 << 5)


class CarmenTranslator
{
public:
	static bool saveCarmenMap(char* fileName, GridMap<float>* gmap, double resolution);
	static bool loadCarmenMap(char* fileName, GridMap<float>* gmap);

	static bool loadBeesoftMap(char* fileName, GridMap<float>* gmap);
	static bool saveBeeSoftMap(char* fileName, GridMap<float>* gmap, double resolution);
	
private:

	static int findRecord(FILE* file, int record);

	static void writeString(FILE* file, char *str, int n);
};



#endif