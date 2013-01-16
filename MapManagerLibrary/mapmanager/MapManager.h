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
/*
This file defines the MapManager class.  This class is the primary class in the Map Manager library.
It handles the loading/saving/conversion of multiple different map file formats.

Once a map is loaded, it is converted to MapManager's internal format.  
There are two storage formats, Grid and Object.  
The Grid storage stores information from grid based maps - simulators such as Carmen and Player/Stage
use this format.  

An object of type GridMapLayer is used to store the grid information.  It maintains
the grid info, and also layer information which can be used to undo/redo any changes to the grid.
The GridMapLayer object also stores the connection between each object (such as a line or rectangle) and 
the grid.  If _viewGridMap is set to true, then each line, rectangle etc that is added to the model
is automatically added to the grid map.  As each new object is added to the model, it is assigned a unique 
object number, or layer.  The GridMapLayer object stores this unique number with each pixel, so that 
if a line/rectangle is removed from the model, it can be correctly removed from the grid also.  
Maintaining this link between the two types of models does come at a memory cost, and if it is not necessary,
then you can call disableLayerInfo(), and it won't happen any more.  However, you will also not be able to
correctly undo/redo additions. 

The objects in the model (line, rectangles, filled rectangles and robots) are stored in a simple List object, the
object is called _listObjects.
Each object in the list is of type LineXYLayer, which has two PointXY objects, a value and a layer.
The value is it's grid value, either -1, or between 0 and 1 inclusive.  The layer value is a unique identifier for the
object. 

*/

#ifndef MAPMANAGER_H
#define MAPMANAGER_H

 
#include "../mapmanager/IBulkJobWorker.h"
#include "SosUtil.h"
#include "../grid/IncludeAll.h"
#include "fstream.h"
#include "GridMapLayer.h"
#include "../fileparsers/IncludeAll.h"
#include "WindowMessaging.h"
#include "../list/SosList.h"
#include "../commonDefs/commonDefs.h"
#include "../logger/Logger.h"

#include <VECTOR>
#include <QUEUE>

#define NUM_UNDO_STEPS					10

#define IS_UNDO_ACTION(action) (action >= ACTION_UNDO_SET_POINT && action<=ACTION_UNDO_REPLACE_OBJECT)

#define MAX_NUM_ROBOT_RUNS				24

#define MAX_DIST_TO_JOIN_VECTOR			0.2


class MapManager : public IBulkJobWorker
{
public:

	//Default constructor for the MapManager class
	MapManager();
	MapManager(GridMap<float> *m);
	MapManager(std::vector<LineXYLayer>* initialVectors, long resolution);
	virtual ~MapManager();

	//Initialise the MapManager with the map pointed to by the object 'm'.
	//If 'performShallowCopy' is set to true, then the internal structures of the
	//map 'm' are taken directly my the MapManager. This essentially wipes the map 'm'
	//clean, but is much faster and more memory efficient.  Use if you don't need the map after
	//calling this method.  If 'performShallowCopy' is false (the default), then a simple
	//copy of the map is done, with the original remaining intact.
	virtual int addMap(GridMap<float> *m, bool performShallowCopy = false);

	//Gets the dimensions of the map in millimetres.  The parameter names are self-explanatory,
	//and the order they are passed in is important.
	virtual bool getDimensions(long& west, long&north,long&east, long&south);

	//Gets the dimensions of the map based on the number of pixels in the grid.
	virtual bool getGridDimensions(long& west, long&north,long&east, long&south);
	
	//sets one point in the map based on an (x,y) position. This is based on a pixel position, not MM.
	virtual void setPoint(long x, long y, float value);

	//sets a line in the map based on MM coordinates, with each pixel passed over by the line
	//being set to the given value.
	virtual void setLine(long x1, long y1, long x2, long y2, float value);

	//adds a robot to the map, with it's centre being at (centreX, centreY), in MM
	LineXYLayer setRobot(long centreX, long centreY);

	//returns the value at the (x,y) position in the grid map (grid coordinates, not MM)
	virtual float getPointVal(long x, long y);

	//fills in a rectangle with the specified value based on two coordinates in MM
	virtual void setRectangleFilled(long x1, long y1, long x2, long y2, float value);

	//sets an unfilled rectangle in the map based on two coordinates in MM
	virtual void setRectangle(long x1, long y1, long x2, long y2, float value);

	//Fills the grid map with the value 'valueToSet'.  This is similar to fill algorithms used
	//by most common paint programs.  the parameters xVal and yVal specify the place in the map
	//to start filling from (in MM).  The 'tolerance' parameter specifies how far from the
	//grid value at position (xval, yVal) a cell can be and still be filled with the new value.
	virtual void fillArea(long xVal, long yVal,float valueToSet,double tolerance);

	//returns true if a map exists. false otherwise
	virtual bool hasMap(){return !(_myMap == 0);}

	//returns true if a voronoi graph has been generated or loaded. False otherwise.
	virtual bool hasVoronoi(){return (_listVoronoiLines.getListSize() > 0);}

	virtual bool hasDelaunay(){return (_listDelaunayLines.getListSize() > 0);}


	//returns true if a path has been generated or loaded. False otherwise.
	bool hasPath(){return (_listPathLines.getListSize() > 0);}

	//returns true if there are any objects in the map, such as lines, rectangles, robots etc. 
	//False otherwise
	bool hasVectors(){return (_listObjects.getListSize() > 0);}

	//returns true if a robot run has been loaded. False otherwise.
	//A robot run displays the path that a robot took in a map.
	bool hasRobotRun(){return (_listRobotRunLists.getListSize() > 0);}

	bool hasRobot();

	//if true is passed to setViewGridMap, then all objects added to the map are automatically
	//integrated into the map as pixels, or cells.  If false is passed to it, then they are not.
	void setViewGridMap(bool viewGridMap);

	void setViewVectorMap(bool viewVectorMap);	
	bool getViewGridMap(){return _viewGridMap;}
	bool getViewVectorMap(){return _viewVectorMap;}

	//enableLayerInfo() causes all objects added to the map to have Layer information maintained
	//which maps their relationship to the grid.  This enables undo/redo functionality, but encurs a memory
	//and performance overhead.
	void enableLayerInfo();

	//turns off the behaviour described over enableLayerInfo()
	void disableLayerInfo();

	//the "clear" functions empty whatever list they refer to
	void clearVoronoi();

	void clearDelaunay();

	void clearPaths();
	void clearVectors();
	void clearGridMap();
	void clearRobotRun();
	bool clearRobots();

	bool undo();//undo the last action
	bool redo();//redo the last undone action

	//Creates a new map with the given dimensions in MM
	bool newMap(long minX, long maxX, long minY, long maxY);
	
	//loads a grid map.  This supports all known grid map types, including MapViewer format,
	//Carmen, Saphira, Beesoft, and an old type no longer used specific to the Grid3D object
	bool loadGridMap(char* filePath);

	//saves a Grid Map in the MapViewer format, in a file with the extension .mvm
	bool saveGridMap(char* fileName);
	
	//Loads a map in the Point List format.  This is an extremely simple file format
	//which should make it easy to write code to parse it.
	bool loadMapAsPointList(char* fileName);
	bool saveMapAsPointList(char* fileName);
	
	//Loads a vector map from a Saphira .wld file.  This map consists of just Lines and at most
	//a single robot. There are no rectangles or grid cells.
	bool loadSaphiraWld(char* fileName);
	bool saveSaphiraWld(char* fileName);
		
	//Loads a voronoi graph from a .vor file
	bool loadVoronoi(char* filePath);

	//Saves a voronoi graph to a .vor file
	bool saveVoronoi(char* filePath);

	//Loads a path from a .pat file
	bool loadPath(char* filePath);

	//Saves a path to a .pat file
	bool savePath(char* filePath);
	
	//Loads a Carmen grid map. This contains only grid information, no vectors
	//The usual file extension is .map
	bool loadCarmen(char* filePath);

	//Saves the current grid information to a Carmen map file.  No vector info is saved
	//The extension .map is appended to the file
	bool saveCarmen(char* filePath);

	//Loads a Beesoft grid map.  No vector info is loaded.
	bool loadBeeSoft(char* filePath);
	//Saves a Beesoft grid map.  No vector info is saved.
	bool saveBeeSoft(char* filePath);

	//Loads a Player/Stage map.  This generally comes in two files, a text file (.world) containing 
	//misc info about the map such as robot position(s), and a PNM image file containing grid information
	//The path to the .world file is expeced in the 'filePath' parameter.  
	//Currently, only Stage WORLDS with a single bitmap image are supported, worlds that contain multiple
	//worlds arranged in some order are not supported, though should be in the future.
	bool loadStageMap(char* filePath);

	//Saves a Player/Stage map.  There are multiple options for how this can be saved, which
	//must be specified by passing in a StageWorldOptions object.
	//The 'occupancyThreshold' parameter specifies at what value cells are considered to be occupied, 
	//and at what value unoccupied - Stage only used a black/white representation, not a graded greyscale.
	bool saveStage(char* filePath,StageWorldOptions& options, float occupancyThreshold = 0.5);
	
	//Loads a map with extension .mvm. This contains all map information that can be stored in the MapManager class,
	//such as Grid info, vector info and robot positions.
	bool loadMapViewerMap(char* filePath);

	//Saves a map in the MapViewer file format.  If 'saveGrid' if true, then grid info is saved.
	//If 'saveVector' is true, vector info is saved.
	bool saveMapViewerMap(char* filePath, bool saveGrid, bool saveVector);

	//Saves a map in the Rossum format.
	bool saveRossum(char* filePath);

	//Loads a picture as grid info. Currently only PNM images are supported.
	bool loadImage(char* filePath);

	//Loads a robot run - this is the path that a robot took in a map.
	bool loadRobotRun(char* filePath);
	
	//Converts a grid map to a vector map using a very simple, lightweight algorithm that fits
	//rectangles around pixels.  The 'threshold' parameter specifies at what value grid cells must be
	//above to be turned into vectors.  
	bool convertGridToLine(float threshold = 0.5);

	//Converts a grid map to a vector map using a combination of the sweep line voronoi algorithm
	//and custom line fitting code.  This is much more CPU and memory intensive than the 
	//convertGridToLine, but generates much better vector approximations of grid maps.
	//The parameters 'minThreshold' and 'maxThreshold' specify the lower and upper boundaries for the
	//cells to be turned into vectors. Only cells between these two values will be used to create vectors.
	//The 'value' parameter specifies the value of each vector when it is applied to the grid.
	//If 'filterByCellValue' is true, then the cells to be vectorised are chosen as explained earlier.
	//If it is false, then the cells are chosen based on the difference between their value and
	//their neighbours value (their contrast).  In this case only the 'minThreshold' parameter is used,
	//and the 'maxThreshold' value is ignored.
	bool convertGridToLineWithVoronoi(float minThreshold, float maxThreshold, 
										bool filterByCellValue = true, float value = 1);

	//Generates a voronoi diagram in the grid map.  All cells in the range [threshold1, threshold2]
	//are considered to be occupied, all other cells are freespace.
	//The diagram can be filtered to remove unwanted edges by setting the 'minDistance' parameter.
	//This specifies the minimum distance that must be between two occupied cells for a voronoi edge
	//to be placed between them.  This is in grid-cell coordinates, and a value of less than 1.5 can result
	//in very bad results, and is not advised, as it places an edge between adjacent occupied cells.
	bool generateVoronoi(float threshold1, float threshold2, float minDistance);

	bool generateDelaunay(float threshold1, float threshold2, float minDistance);


	//Averages the currently loaded grid map with the other grid map.
	//This keeps a running average of grid maps, so if it is called, say, 4 times, it will
	//come up with a grid average value per cell of (a + b + c + d)/4, 
	//rather than ( ((((a + b)/2) + c)/3) + d)/4).
	bool averageGridMap(char* filePath);

	//Calcualtes a correlation coefficient between the currently loaded grid map and the
	//map in the file 'filePath'.  This is a value between 0 and 1, with the higher the value
	//the more similar the two maps are
	double correlateMap(char* filePath);

	//Calculates the dissimilarity of two maps based on the Map Score benchmark by Martin and Moravec.
	//The min value is 0 (meaning a perfect match) with no max value.  The higher the value, the 
	//less similar the maps.  This value is not normalised, and therefore is less useful than correlateMap(),
	//but is useful for comparing a number of maps against a single perfect map (good in unsupervised learning)
	double mapScoreMap(char* filePath,bool justCompareOccAreas = false);
	
	//Returns the resolution of the map, that is the width or height of a single square cell in millimetres
	long getResolution(){return _resolution;}

	//Set the resolution of the maps - i.e. the width or height of a single square cell in millimetres.
	//This causes all vectors to be removed from the GridMapLayer object and readded at a different resolution.
	//It is only advisable to use this early on, soon after the map has been created/loaded, otherwise it can have
	//unexpected results if you have been editing both grid and vector objects.
	void setResolution(long res);
	
	//Generates a Configuration Space in the grid map.  This basically expands all occupied cells 
	//(cells between the min and max values) by the given distance (in MM).
	bool generateCSpaceSimple(float min, float max, long distance);

	//Reduces the grid map to values 0 and 1.  All cells with a value between 'min' and 'max' are set to 1.
	//All others are set to 0
	bool thresholdMap(float min, float max);

	//This crops the map to the given coordinates in MM.  All grid cells outside these coordinates are wiped
	//clean (set to 0), and all vectors outside this area either deleted (if they are completely outside), or
	//clipped to finish at the boundary (if partially in/out of the area)
	bool cropMap(long x1,long y1, long x2, long y2);
	
	//Performs a filtering operation of the map that wipes out all cells between the 'min' and 'max'
	//values that are surrounded by 3 or more cells not in that range.  The cell's new value is an average
	//of the cells around it.
	bool smoothMap(float min, float max);

	
	//flip all values of the grid map. If a value was 0, it will be 1, 0.8 ->0.2 etc
	//Values of -1 stay the same
	bool negativeMap();

	//translates (or moves) the map in the X and/or Y directions by 'xDist' and 'yDist' millimetres
	//One limitation of this is that it can only be translated in whole grid cells, so for example
	//if the resolution is 100, then the values 5000 and 1200 would be acceptable, but 223 and 6669 would not be.
	bool translateMap(long xDist, long yDist);

	//Does a full copy of the current grid map into the map pointed to by 'mapToCopyInto'
	bool getLatestGridMap(GridMap<float>* mapToCopyInto);

	//Copies all the objects currently in the map into the queue object 'listToCopyInTo'
	bool getAllObjects(std::queue<LineXYLayer>& listToCopyInTo);
	bool getRobots(std::queue<LineXYLayer>& listToCopyInTo);

	bool getErrorStrings(char* title, char* errorString);

	//initialises all internal structures. It's not recommended to call this.
	virtual void		init();

	//Resets all internal structures
	virtual void		resetAllObjects();

	//Converts grid coordinates to millimetre coordinates
	void				gridToMm(float gridX, float gridY, long& mmX, long& mmY);

	//Converts millimeter coordinates to grid coordinates
	void				mmToGrid(long mmX, long mmY,float& gridX, float& gridY);

	//Returns true if the object 'obj' is inside the area between 'topLeft' and 'bottomRight'.  
	//If the object is only partially inside the area, 'partiallyInRect' is set to true.
	bool				objectInRect(const LineXYLayer & obj, PointXYLong topLeft,PointXYLong bottomRight, bool& partiallyInRect);
	
	//Deletes the object with the layer value 'objectNumber'
	bool				deleteObject(long objectNumber);

	//Returns true if the object with layer value 'objectNumber' exists, and the object is copied into the 
	//object 'obj'
	bool				getObject(long objectNumber, LineXYLayer& obj);

	bool				replaceObject(long objectNumber, const LineXYLayer& object);

	bool				translateObjects(const std::vector<long>& objectNumbers, long xDist, long yDist, std::vector<long>& newObjectNumbers);

	//this sets the size of a robot object to a predefined size
	LineXYLayer			fixRobotSize(LineXYLayer object);

	//This iterates through all the objects in the map to find the most outlying vectors.
	void				refreshVectorBoundary();

	//creates a new layer, which is called once every time a new object is added to the map,
	//and the grid map is being used.
	long				getNextLayer();
	
	//this method is used for objects not yet added to the gridmap
	long				getNextNegativeLayer();
	
	void cancelBulkJob();
	bool jobCompletedSuccessfully();

	enum
	{
		FILE_TYPE_MAPVIEWER,
		FILE_TYPE_STAGE,
		FILE_TYPE_SAPHIRA,
		FILE_TYPE_BMP,
		FILE_TYPE_JPEG,
		FILE_TYPE_ROSSUM,
		FILE_TYPE_CARMEN,
		ROBOT_RUN_MV_FORMAT,
		ROBOT_RUN_STAGE_FORMAT,
		ACTION_SET_POINT,
		ACTION_SET_OBJECT,
		ACTION_DELETE_POINT,
		ACTION_DELETE_OBJECT,
		ACTION_REPLACE_OBJECT,
		ACTION_UNDO_SET_POINT,
		ACTION_UNDO_SET_OBJECT,
		ACTION_UNDO_DELETE_POINT,
		ACTION_UNDO_DELETE_OBJECT,
		ACTION_UNDO_REPLACE_OBJECT
	};

	//Returns a reference to an object that can be used to read all the objects (vectors and robots) in the map.
	//You cannot alter the contents using this reference.
	IListReader<LineXYLayer>*	getAllObjectsReader();

	//Returns a reference to an object that can be used to read all the Voronoi lines (edges) in the map.
	//You cannot alter the contents using this reference.
	IListReader<LineXY>*		getVoronoiLinesReader();

	//Returns a reference to an object that can be used to read all the Voronoi vertices (places where edges meet)
	//in the map.
	//You cannot alter the contents using this reference.
	IListReader<PointXY>*		getVoronoiVerticesReader();

	//Returns a reference to an object that can be used to read all the Delaunay lines (edges) in the map.
	//You cannot alter the contents using this reference.
	IListReader<LineXY>*		getDelaunayLinesReader();
	
	//Returns a reference to an object that can be used to read all the Path lines in the map.
	//You cannot alter the contents using this reference.
	IListReader<LineXYLong>*	getPathLinesReader();

	//Returns a reference to an object that can be used to read a grid one row at a time
	ICopyRow2D<float>*			getGridReader();

protected:
	//Private methods

	//Sets the error strings.  This is called when a method finds an error.  The error strings can be
	//retrieved by the calling class by calling the getErrorStrings() method
	void				setErrorStrings(char* title, char* errorString);

	//Sets the error strings, and concatenates 'errorString1' and 'errorString2'
	void				setErrorStrings(char* title, char* errorString1,char* errorString2);
	
	//Incorporates all vectors into the grid that have not already been added to the grid 
	void				pushAllVectorsOntoGrid();
	
	//copies the dimensions of the vectors (objects in the _listObjects list) to the GridMapLayer object
	inline void			copyVectorDimensionsToGrid();	
	
	//adds a new object to the map.  The type of the object is in the 'type' field of the LineXYLayer object
	void				setObject(LineXYLayer& object);

	//add a new user action.  This info is used when undoing user actions
	void				addUserAction(int action, bool clearUndoneArtifacts = true);

	//add a new undo action.  This info is used when redoing the last thing a user undid
	void				addUndoAction(int action);

	//Removes the last undo action from the _listUndoActions list.  This is used to keep the size of the 
	//undo list under a specified size
	void				removeOldestUndoAction();

	//converts a LineXYLong object to a LineXY object
	LineXY				LineXYLongToFloat(LineXYLong l, bool isRectangle);

	//clears all undo information
	void				resetUndoInfo();

	//Private data members
	GridMap<float>*					_myMap;
	long							_resolution;

	bool							_viewGridMap;
	bool							_viewVectorMap;
	bool							_viewVoronoi;
	bool							_viewVoronoiVertices;
	bool							_viewPath;
	bool							_viewBackgroundGrid;
	bool							_viewRobotRun;
	bool							_viewRobot;
	bool							_viewBorder;
	int								_viewCellMode;
	
	long							_mapMinX, _mapMaxX, _mapMinY,_mapMaxY;	

	int								_mapAverageCount;

	List<LineXYLayer>				_listObjects;
	List<LineXYLayer>				_listObjectsDeleted;
	List<LineXYLayer>				_listObjectsUndone;

	//This list contains all the objects that were replaced by other objects
	//Each entry in the list contains the layer number of the new object, 
	//as well as the complete object it replaced in the 'value' field
	List<LayerValue<LineXYLayer> >	_listObjectsReplaced;
	List<LayerValue<LineXYLayer> >	_listObjectsReplacedUndone;


	List<long>						_listPoints;		//list of layers of points - the layer number is stored
	List<long>						_listPointsUndone;	//list of undone layers of points - the layer number is stored

	List<int>						_listUserActions;
	List<int>						_listUndoActions;

	bool							_pointsAddedToUserActions;

	GridMapLayer					_gridLayer;
	long							_latestLayer, _latestNegLayer;
	bool							_gotNegLayer;

	bool							_gridChanged,_vectorsChanged,_voronoiChanged;

	LineXYLong						_vectorBoundary;	

	List<LineXY>					_listVoronoiLines;//stores all the small lines in a voronoi diagram
	List<PointXY>					_listVoronoiVertices;//stores all the vertices in the voronoi diagram
	List<LineXY>					_listVoronoiEdges;//stores the links between the vertices in the voronoi diagram

	List<LineXY>					_listDelaunayLines;//stores all the small lines in a Delaunay diagram
	
	List<LineXYLong>				_listPathLines;
	List<LineXYLong>				_listPathGoalPoints;

	long							_robotRadius;
	
	List<List<PointXYLong>* >		_pathLists;

	ListUnordered<ListUnordered<SosPose>* >		_listRobotRunLists;

	char							_errorBuffer[2][1000];
	bool							_hasError;

	bool							_bulkOperationCancelled;
	bool							_bulkOperationSuccessful;
	
	DEF_LOG
};


#endif