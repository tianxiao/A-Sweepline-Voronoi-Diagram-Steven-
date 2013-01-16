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


#include <fcntl.h>
#include <stdio.h>
#include <time.h>
#include "CarmenTranslator.h"


void CarmenTranslator::writeString(FILE* fp, char *str, int n)
{
  int l, i;

  if (str == NULL)
    l = 0;
  else
    l = strlen(str);
  
  if(l > n)
    l = n;
  for(i = 0; i < l; i++)
    fprintf(fp, "%c", str[i]);
  if(l < n)
    for(i = l; i < n; i++)
      fprintf(fp, " ");
}

bool CarmenTranslator::saveCarmenMap(char* fileName, GridMap<float>* gmap, double resolution)
{
	int x, record_size;
	float local_resolution = 100;//TO DO : must be changed

	FILE *stream;

	if(gmap == 0)
		return false;
	
	/* Open file in text mode: */
	if( (stream = fopen( fileName, "w" )) == NULL )
		return false;//could not open file

	//switch to binary mode
	_setmode(_fileno(stream),_O_BINARY);
	
	float *arr = 0;
	int sizeX = 0, sizeY = 0;

	long west = gmap->getUpdatedDimensions(WEST), east = gmap->getUpdatedDimensions(EAST),
		north = gmap->getUpdatedDimensions(NORTH), south = gmap->getUpdatedDimensions(SOUTH);
	
	sizeX = gmap->getMapWidth();
	sizeY = gmap->getMapHeight();

	time_t t = time(NULL);

	fprintf(stream, "#####################################################\n");
	fprintf(stream, "#\n");
	fprintf(stream, "# Carnegie Mellon Robot Toolkit (CARMEN) map file\n");
	fprintf(stream, "#\n");  
	fprintf(stream, "# Creation date : %s", asctime(localtime(&t)));
	fprintf(stream, "# Map size      : %d x %d\n", sizeX, sizeY);
	fprintf(stream, "# Resolution    : %.1f\n", resolution);
	fprintf(stream, "# Origin        : %s\n", "MapViewer by Shane O'Sullivan");
	fprintf(stream, "# Description   : %s\n", "http://mapviewer.skynet.ie");
	fprintf(stream, "#\n");
	fprintf(stream, "#####################################################\n");

	fprintf(stream, "%s%s", LABEL, VERSION);

	int recordSize2 = 10 + 10 + sizeof(time_t) + 80 + 80;
	
	fputc(CREATOR_RECORD, stream);

	fwrite(&recordSize2, sizeof(int), 1, stream);
	fprintf(stream, "CREATOR   ");
	writeString(stream, "UNKNOWN", 10);

	t = time(NULL);
	fwrite(&t, sizeof(time_t), 1, stream);
	writeString(stream, "MapViewer by Shane O'Sullivan ", 80);
	writeString(stream, "http://mapviewer.skynet.ie ", 80);
	
	arr = new float[sizeY];
	
	fputc(GRIDMAP_RECORD, stream);
	
	record_size = 10 + 12 + sizeX * sizeY * 4;
	fwrite(&record_size, sizeof(int), 1, stream);
	fprintf(stream, "GRIDMAP   ");
	fwrite(&sizeX, sizeof(int), 1, stream);
	fwrite(&sizeY, sizeof(int), 1, stream);
	fwrite(&local_resolution, sizeof(float), 1, stream);
	for(x = 0; x < sizeX; x++)
	{
		for(int y = 0; y < sizeY; y++)
		{
			arr[y] = gmap->getGridRef(west + x,south + y);
		}
		fwrite(arr, sizeof(float) * sizeY, 1, stream);
	}

	fclose(stream);
	
	return true;
}


bool CarmenTranslator::loadCarmenMap(char* fileName, GridMap<float>* gmap)
{
	GET_FILE_LOG_GLOBAL

	if(gmap == 0 || fileName == 0)
	{
		LOG<<"Returning false 1"<<endl;
		return false;
	}
	
	FILE *stream;
	int record_type = 0, record_size = 0;
	char record_description[12];
	int size_x = 0, size_y = 0;
	float resolution = 0;
	

	float** mapCast = 0;
	float*  mapArray = 0;
	
	if((stream = fopen( fileName, "r" )) == NULL )
	{	
		LOG<<"Failed to open the file";
		return false;//could not open the file
	}

	_setmode(_fileno(stream),_O_BINARY);
	
	if(findRecord(stream,GRIDMAP_RECORD) < 0)
	{
		LOG<<"Could not find a grid map";
		fclose(stream);
		return false;
	}
	
	record_type = fgetc(stream);
	fread(&record_size, sizeof(int), 1, stream);
	fread(record_description, 10, 1, stream);
	fread(&size_x, sizeof(int), 1, stream);
	fread(&size_y, sizeof(int), 1, stream);
	fread(&resolution, sizeof(float), 1, stream);

	mapArray = new float[size_x * size_y];
	mapCast  = new float*[size_x];//(float **)calloc(size_x, sizeof(float *));

	for(int n = 0; n < size_x; n++)
		mapCast[n] = mapArray + n * size_y;

	fread(mapArray, sizeof(float) * size_x * size_y, 1, stream);

	fclose(stream);

	//now copy the 2D array into the GridMap

	for(int x = 0; x< size_x; x++)
	{
		for(int y = 0; y< size_y; y++)
		{
			if(mapCast[x][y] > 0 && mapCast[x][y] < 0.001)
				mapCast[x][y] = 0;

			if(mapCast[x][y] < -1)
				mapCast[x][y] = -1;

			if(mapCast[x][y] < 0 && mapCast[x][y] > -0.001)
				mapCast[x][y] = 0;

			if(mapCast[x][y] > 1)
				mapCast[x][y] = 1;

			gmap->updateGridRef(mapCast[x][y],x,y);
		}
	}

	delete [] mapArray;
	delete [] mapCast;

	
	return true;
}


bool CarmenTranslator::loadBeesoftMap(char* fileName, GridMap<float>* gmap)
{	

	char strArray2[4][40] = {"robot_specifications->global_mapsize_x",
							"robot_specifications->global_mapsize_y",
							"robot_specifications->resolution",
							"global_map[0]:"};

	char **strArray = 0;
	strArray = new char*[4];
	for(int j = 0; j<4; j++)
	{
		strArray[j] = strArray2[j];
	}

	GET_FILE_LOG_GLOBAL	
	
	long x_index, y_index;
	float value = 0;
	char buf[1024];
	float resolution;	
	long y_size = 0,x_size = 0;	

	bool foundResolution = false;	

	ifstream in;
	in.open(fileName);
	if(!in.is_open())
		return false;

	int entryType = -1;
	bool fileIsBad = false;
	long val = 0;
	
	for(int i = 0; i< 4 && !fileIsBad; i++)
	{
		in>>buf;	
		LOG<<"Got entry "<<buf<<endl;
		entryType = SosUtil::stringInArray(buf,strArray,4);

		LOG<<"buf is of type "<<entryType<<endl;

		switch(entryType)
		{
		case 0://X size - very strange, beesoft seems to reverse x and y values
			in>>y_size;
			break;
		case 1://Y size - very strange, beesoft seems to reverse x and y values
			in>>x_size;	
			break;
		case 2://resolution
			in>>resolution;
			break;
		case 3://global map
			in>>val>>val;//not doing anything with this right now
			break;
		default:
			fileIsBad = true;
			break;
		}
	}

	if(fileIsBad)
	{
		in.close();
		delete[] strArray;
		return false;
	}

	LOG<<"Got y_size = "<<y_size<<", and x_size = "<<x_size<<endl;
	//*Logger::getFileOstream()<<"Got y_size = "<<y_size<<", and x_size = "<<x_size;

	
	for (x_index = 0; x_index < x_size; x_index++) 
	{
		for (y_index = 0; y_index < y_size; y_index++) 
		{
			in>>value;

			if (value >= 0)
				value = 1- value;
			else
				value = -1;
			gmap->updateGridRef(value,x_index,y_index);
		}
	}
	
	in.close();
	delete[] strArray;

	return true;		
}

bool CarmenTranslator::saveBeeSoftMap(char* fileName, GridMap<float>* gmap, double resolution)
{
	if(fileName == 0 || strlen(fileName) == 0 || gmap == 0)
		return false;

	ofstream out;
	out.open(fileName);

	//return false if we couldn't open the file
	if(!out.is_open())
		return false;

	long width = gmap->getMapWidth(), height = gmap->getMapHeight();

	out<<"robot_specifications->global_mapsize_x  "<<height<<endl;
	out<<"robot_specifications->global_mapsize_y  "<<width<<endl;
	out<<"robot_specifications->resolution  "<<resolution<<endl;
	out<<"global_map[0]: "<<width<<' '<<height<<endl;

	float val = 0;

	long minX=0, maxX=0,minY=0,maxY=0;
	gmap->getAllUpdatedDimensions(minX,maxY,maxX,minY);

	for (long x = minX; x <= maxX; x++) 
	{
		for (long y = minY; y <= maxY; y++) 
		{
			val = gmap->getGridRef(x,y);
			
			if (val >= 0)
				val = 1- val;
			else
				val = -1;
			out<<val<<' ';			
		}
		out<<endl;
	}

	return true;
}

int CarmenTranslator::findRecord(FILE* fp, int specific_chunk)
{
	GET_FILE_LOG_GLOBAL
	int record_type, record_size, done = 0;
	
	fseek(fp, 0, SEEK_SET);

	char comment_str[100], c;
	char *err, id[1024] = {0};
	
	do {
		c = fgetc(fp);
		
		if(c == EOF)
		{
			return -1;
		}
		else if(c == '#') 
		{
			err = fgets(comment_str, 100, fp);
			if(err == NULL)
				return -1;
		}
		else
			fseek(fp, -1, SEEK_CUR);
	} while(c == '#');
	
	if(fread(&id, strlen(LABEL)+strlen(VERSION), 1, fp) == 0)
		return -1;

	if(strncmp(id, LABEL, strlen(LABEL)) != 0)
	{
		return -1;
	}

	if(strncmp(id+strlen(LABEL), VERSION, strlen(VERSION)) != 0)
		return -1;
	
	do {
		record_type = fgetc(fp);
		
		if(record_type == EOF)
			done = 1;
		if(record_type == specific_chunk) {
			fseek(fp, -1, SEEK_CUR);
			return 0;
		}
		if(!done)
		{
			if(fread(&record_size, sizeof(int), 1, fp) < 1)
				done = 1;
		}
		if(!done)
		{
			if(fseek(fp, record_size, SEEK_CUR) < 0)
				done = 1;
		}
		if(!done && feof(fp))
		{
			done = 1;
		}
	} while(!done);
	return -1;
}

