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

#ifndef WINDOWMESSAGING_H
#define WINDOWMESSAGING_H

#include <string.h>

static char WindowMessagingBuffer[1024] = {0};

#define WMB(str) strcpy(WindowMessagingBuffer,str)

#define PARSE_ERROR_LINE WMB("Parse error in line: ")
#define PARSE_ERROR_IN_LINE(num) strcat(PARSE_ERROR_LINE, ltoa(num,WindowMessagingBuffer,10))
#define PARSE_ERROR_TOKEN WMB("Parse error on token: ")
#define PARSE_ERROR_ON_TOKEN(str) strcat(PARSE_ERROR_TOKEN, str)

#define STD_ERROR_TITLE				"Error"
#define STD_CONFIRM_TITLE			"Wait!"
#define STD_NO_MAP_LOADED			"No Map Loaded"
#define STD_NO_FILE_NAME			"No file name specified"
#define STD_NOT_ENOUGH_MEMORY		"Not enough memory to perform operation"
#define STD_NO_IMAGE_FILE_NAME		"No image file name specified"
#define STD_ONE_BITMAP_SUPPORTED	"Only one bitmap per world is supported"
#define STD_GZ_NOT_SUPPORTED		"Zip Files not supported. Extract the image first, change the WORLD file and try again."

#define ERROR_FILE_OPEN(filename) strcat(WMB("Could not open file: "),filename)
#define ERROR_FILE_SAVE(filename) strcat(WMB("Could not save file: "),filename)

#define ERROR_REACHED_MAX(obj) strcat(WMB("Error, reached max number of "),obj)

#define ERROR_NO_SONAR_READINGS WMB("No Sonar readings in file")



#endif