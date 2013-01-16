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

#ifndef SOSUTIL_H
#define SOSUTIL_H

#include <math.h>
#include <iostream.h>
#include <fstream.h>
#include <string.h>
#include <stdio.h>
#include "../logger/Logger.h"

#ifdef WIN32
//#include <winbase.h>
#endif


#ifndef PI
#define PI 3.1415927
#endif

#define FORX(init, val) for(long x = init; x<val; x++)
#define FORY(init, val) for(long y = init; y < val; y++)
#define FORX0(val) FORX(0,val)
#define FORY0(val) FORY(0,val)

enum
{	
	OBJECT_TYPE_UNDEFINED,
	OBJECT_TYPE_LINE,
	OBJECT_TYPE_RECTANGLE,
	OBJECT_TYPE_RECTANGLE_FILLED,
	OBJECT_TYPE_POINT,
	OBJECT_TYPE_COLLECTION,
	OBJECT_TYPE_ROBOT
};

#define ROBOT_RADIUS					220

class LineNode;
class LineObj;
class PointXY;
class PointXYLong;
class LineXY;
class LineXYFloat;
class LineXYLong;

class SosUtil
{
public:

	static void init();
	static double getDist(double x1, double y1, double x2, double y2);
	static double getDist(long x1, long y1, long x2, long y2);
	static double getDist(PointXY pt1, PointXY pt2);
	static double getDist(PointXYLong pt1, PointXYLong pt2);
	
	static bool getCircleCentreFrom3Points(double x1, double y1, double x2, double y2, double x3, double y3,
		double &centreX, double &centreY);
	
	static void pointRotate(double *x, double *y, double th);
	
	static double degToRad(double deg) ;
	static double radToDeg(double rad) ;
	static double atan2(double y, double x) ;
	
	static PointXY getMidPoint(LineXY);
	static PointXYLong getMidPoint(LineXYLong);
	
	static double fixAngle(double angle) ;
	
	static double cos(double angle); 	
	static double sin(double angle);
	
	static void SosUtil::sinAndCos(double angle, double& sine, double& cosine);

	static double roundDown(double num);
	static double roundUp(double num);
	static double fabs(double num);
	static float fabs(float num);
	static long fabs(long num);

	static long getRandomNumber(long min, long max);
	static double getRandomNumber(double min, double max, int precision);
	static bool getRandomBool();

	static void generateRandomDistribution(float* arr,int size, float average);
	static void generateRandomDistributionWithMin(float* arr,int size, float average, float min);
	
	static double minVal(double num1, double num2);
	static double maxVal(double num1, double num2);
	static float minVal(float num1, float num2);
	static float maxVal(float num1, float num2);
	static long minVal(long num1, long num2);
	static long maxVal(long num1, long num2);   

	static long minVal(long num1, long num2, long num3);
	static long maxVal(long num1, long num2, long num3);

	static void swap(long& num1, long& num2);
	static void swap(double& num1, double& num2);
	static void swap(float& num1, float& num2);
	
	
	static bool ensureSmaller(long &num1, long &num2);
	static bool ensureSmaller(double &num1, double &num2);
	static bool ensureSmaller(float &num1, float &num2);
	
	static bool between(double num, double lowerVal, double upperVal);
	static bool between(long num, long lowerVal, long upperVal);
	
	static float midWay(float num1, float num2);
	
	static char* doubleToString(double, int, char*);
	static char* floatToString(float val, int precision, char* displayString);
	static bool is_numeric(char * str) ;
	static bool is_integer(char * str) ;
	static bool endsWith(char* str1, char* ending);
	static void getBaseFileName(char* source, char* dest);
	static bool getPath(char* source, char* dest);
	static bool getFilenameAndPathNoExtension(char* source, char* dest);

	static float setPrecision(float num,int pres);
	
	static void sleep(unsigned int ms);
	
	static LineNode* pushStack(LineNode* start, LineNode* newItem);
	static void deleteList_Iterative(LineNode* node);
	static LineNode* deleteNode(LineNode* start,LineNode* nodeToDelete);
	
	static char* stripPath(char* str);
	
	static bool readNum(FILE* file, int& num);
	static bool readNum(FILE* file, long& num);
	static bool readNum(FILE* file, float& num);
	
	static int stringInArray(char* str, char** strArray, int arrSize);
	static int stringInArrayNoCase(char* str, char** strArray, int arrSize);

	//finds the first instance of the "key" string in the "string" variable
	//returns -1 if not found
	static int firstInstance(const char* string, const char* key, bool ignoreCase=false);
	
	//trims the spaces at the start and end of the string
	static bool trim(const char* source, char* dest);

	//this gets a token from a string, and returns the last position of the end of the returned token
	//If the token 'tokenNum' doesn't exist, -1 is returned.
	static int getToken(const char* source, char* token, const char* delimiters, int numDelimiters, int tokenNum);

	static int tokenise(const char* source, char** tokens, const char* delimiters, 
					  int numDelimiters, int firstTokenNum, int lastTokenNum);

	//returns true if the two TEXT files are equal, false if they aren't
	static bool compareFiles(char* file1, char* file2);
	
private:
	SosUtil();//don't let anyone instatiate an instance of the class
	static bool _randInitialised;

	static bool _initialised;
	
};

class PointXYLong;

class PointXY
{
public:
	bool operator==(const PointXY& node) const{return (node.x==x&&node.y==y);}
	bool operator!=(const PointXY& node) const{return !(node.x==x&&node.y==y);}
	bool operator<(const PointXY& node) const{return ((x+y) < (node.x+node.y));}
	bool operator>(const PointXY& node) const{return ((x+y) > (node.x+node.y));}
	PointXY operator=(const float num){x = y = num; return *this;}
	PointXY operator=(const PointXYLong num);
	friend ostream &operator<<(ostream &out, const PointXY& outPoint);
	
	PointXY(){x = y = 0;}
	PointXY(float xx, float yy){x = xx; y = yy;}
	void setPoints(float xx,float yy){x = xx;y = yy;}
	float x;
	float y;
};

class PointXYLong
{
public:
	bool operator==(const PointXYLong& node) const{return (node.x==x&&node.y==y);}
	bool operator!=(const PointXYLong& node) const{return !(node.x==x&&node.y==y);}
	bool operator<(const PointXYLong& node) const{return ((x+y) < (node.x+node.y));}
	bool operator>(const PointXYLong& node) const{return ((x+y) > (node.x+node.y));}
	PointXYLong operator=(const long num){x = y = num; return *this;}
	PointXYLong operator=(const PointXY num){x=(long)num.x; y = (long)num.y; return *this;}
	
	friend ostream &operator<<(ostream &out, const PointXYLong& outPoint);

	PointXYLong(){x = y = 0;}
	PointXYLong(long xx, long yy){x = xx; y = yy;}
	void setPoints(long xx,long yy){x = xx; y = yy;}
	long x;
	long y;
};


/*
class PointXYFloat
{
public:
bool operator==(const PointXYFloat& node) const{return (node.x==x&&node.y==y);}
bool operator!=(const PointXYFloat& node) const{return !(node.x==x&&node.y==y);}
bool operator<(const PointXYFloat& node) const{return ((x+y) < (node.x+node.y));}
bool operator>(const PointXYFloat& node) const{return ((x+y) > (node.x+node.y));}
PointXYFloat operator=(const float num){x = y = num; return *this;}

  PointXYFloat(){x = y = 0;}
  PointXYFloat(float xx, float yy){x = xx; y = yy;}
  float x;
  float y;
  };
*/

class PointXYZ
{
public:
	bool operator==(const PointXYZ& node) const{return (node.x==x&&node.y==y);}
	bool operator!=(const PointXYZ& node) const{return !(node.x==x&&node.y==y);}
	bool operator<(const PointXYZ& node) const{return (value < node.value);}
	bool operator>(const PointXYZ& node) const{return (value > node.value);}
	
	PointXYZ(){x = y = 0;value = 0;}
	PointXYZ(long xx,long yy,float val){x = xx;y=yy;value=val;}
	
	long x;
	long y;
	float value;
};

class LineXY
{
public:
	bool operator==(const LineXY& node) const
	{
		return (pt1==node.pt1&&pt2==node.pt2) ||(pt1==node.pt2&&pt2==node.pt1);
	}
	bool operator!=(const LineXY& node) const{return !(*this==node);}
	bool operator<(const LineXY& node) const{return false;}
	bool operator>(const LineXY& node) const{return false;}
	LineXY operator=(const float num){pt1 = pt2 = num; return *this;}
	friend ostream &operator<<(ostream &out, const LineXY& outLine);
	
	LineXY()
	{
		pt1 = pt2 = 0;
		GET_FILE_LOG
			//	LOGGING_OFF
	}
	LineXY(float x1, float y1,float x2, float y2)
	{
		pt1.x = x1; pt1.y = y1;pt2.x=x2;pt2.y=y2;
		GET_FILE_LOG
	}
	LineXY(PointXY p1, PointXY p2)
	{
		pt1 = p1; pt2 = p2;
		GET_FILE_LOG
			//	LOGGING_OFF
	}
	LineXY(PointXYLong p1, PointXYLong p2)
	{
		pt1 = p1; pt2 = p2;
		GET_FILE_LOG
			//	LOGGING_OFF
	}
	
	void setPoints(float x1, float y1,float x2, float y2){pt1.x=x1;pt2.x=x2;pt1.y=y1;pt2.y=y2;}
	double getPerpDistTo(float x, float y);
	void setLineByAngle(double midPointX, double midPointY, double LineAngle, double length );
	double getAngle();
	bool getIntersection(LineXY line2, double &x, double &y, bool linesAreInfinite,double threshold=2 );
	
	
	PointXY pt1,pt2;
	DEF_LOG
};


class LineXYLong
{
public:
	bool operator==(const LineXYLong& node) const
	{
		return (pt1==node.pt1&&pt2==node.pt2) ||(pt1==node.pt2&&pt2==node.pt1);
	}
	bool operator!=(const LineXYLong& node) const{return !(*this==node);}
	bool operator<(const LineXYLong& node) const{return false;}
	bool operator>(const LineXYLong& node) const{return false;}
	friend ostream &operator<<(ostream &out, const LineXYLong& outLine);
	

	LineXYLong operator=(const long num){pt1 = pt2 = num; return *this;}
	void setLineByAngle(double midPointX, double midPointY, double LineAngle, double length );
	double getPerpDistTo(long x, long y);
	double getAngle();
	bool getIntersection(LineXYLong line2, double &x, double &y, bool linesAreInfinite );
	
	
	LineXYLong()
	{
		pt1 = pt2 = 0;
		//GET_FILE_LOG
		LOGGING_OFF
	}
	LineXYLong(long x1, long y1,long x2, long y2)
	{
		pt1.x = x1; pt1.y = y1;pt2.x=x2;pt2.y=y2;
		GET_FILE_LOG
	}
	LineXYLong(PointXY p1, PointXY p2)
	{
		pt1 = p1; pt2 = p2;
		//	GET_FILE_LOG
		//	LOGGING_OFF
	}
	LineXYLong(PointXYLong p1, PointXYLong p2)
	{
		pt1 = p1; pt2 = p2;
		//	GET_FILE_LOG
		//	LOGGING_OFF
	}
	
	void setPoints(long x1, long y1,long x2, long y2){pt1.setPoints(x1,y1);pt2.setPoints(x2,y2);}
	
	
	PointXYLong pt1,pt2;
	DEF_LOG
};

class LineXYLayer : public LineXY
{
public:
	friend ostream &operator<<(ostream &out, const LineXYLayer& outLine);
	
	LineXYLayer(){layer = 0;value = 0;}
	LineXYLayer(long layerNum,float val,short objType, float x1, float y1,float x2, float y2):LineXY(x1,y1,x2,y2)
	{
		layer = layerNum;
		value = val;
		type = objType;
	}
	
	bool operator==(const LineXYLayer& node) const{return (layer == node.layer);}
	bool operator!=(const LineXYLayer& node) const{return !(layer == node.layer);}
	bool operator<(const LineXYLayer& node) const{return (layer < node.layer);}
	bool operator>(const LineXYLayer& node) const{return (layer > node.layer);}
	LineXYLayer operator=(const float num){pt1 = pt2 = num; return *this;}
	LineXYLayer operator=(const LineXY num){pt1=num.pt1;pt2 = num.pt2; return *this;}
	
	long layer;
	float value;
	short type;
};


template<class T>
class LayerValue
{
public:
	LayerValue()
	{
		//GET_FILE_LOG
		///	LOGGING_OFF
		//LOG<<"LayerValue() empty constructor";
		layerNumber = 0;
		value = 0;		
	}
	LayerValue(long layer, T val,bool deleteObject = true)
	{
		//GET_FILE_LOG
		//	LOGGING_OFF
		//	LOG<<"LayerValue(...) non empty constructor ENTRY"<<endl;
		//LOG<<"LayerValue("<<layer<<", val) with val";
		layerNumber = layer; 
		value = val;
		//	LOG<<"LayerValue(...) non empty constructor EXIT"<<endl;
	}
	
	
	bool operator==(const LayerValue& node) const{return (layerNumber == node.layerNumber);}
	bool operator!=(const LayerValue& node) const{return !(layerNumber == node.layerNumber);}
	bool operator<(const LayerValue& node) const{return ((layerNumber) < (node.layerNumber));}
	bool operator>(const LayerValue& node) const{return ((layerNumber) > (node.layerNumber));}
	bool operator==(const long& num) const{return (layerNumber == num);}
	bool operator!=(const long& num) const{return !(layerNumber == num);}
	bool operator<(const long& num) const{return ((layerNumber) < num);}
	bool operator>(const long& num) const{return ((layerNumber) > num);}
	void operator=(const LayerValue node)
	{	layerNumber = node.layerNumber;
	value = node.value; 
	}
	
	long layerNumber;
	T value;
	
private:
	
	//	DEF_LOG
};


class PathNode
{
public:
	bool operator==(const PathNode& node) const{return (node.x==x&&node.y==y);}
	bool operator!=(const PathNode& node) const{return !(node.x==x&&node.y==y);}
	bool operator<(const PathNode& node) const{return (value < node.value);}
	bool operator>(const PathNode& node) const{return (value > node.value);}
	
	PathNode(){x = y = value = 0; next = prev = 0;}
	PathNode(double xx,double yy,double val){x = xx;y=yy;value=val;next=prev=0;}
	
	double x;
	double y;
	double value;
	
	PathNode* next;
	PathNode* prev;
};

class SosPose
{
public:
	bool operator==(const SosPose& node) const{return (node._x==_x&&node._y==_y&&node._th==_th);}
	bool operator!=(const SosPose& node) const{return !(node==*this);}
	bool operator<(const SosPose& node) const{return false;}//((_x<node._x)||(_y<node._y));}
	bool operator>(const SosPose& node) const{return false;}//((_x>node._x)||(_y>node._y));}
	SosPose operator=(const double num){_x = _y = _th = num; return *this;}
	
	SosPose(double x = 0, double y = 0, double th = 0)
    { 
		_x = x; 
		_y = y; 
		_th = th; 
	}
	
	SosPose(const SosPose &pose) : _x(pose._x), _y(pose._y), _th(pose._th) 
	{
	}
	
	
	virtual ~SosPose() 
	{
	}
	
	virtual void setPose(double x, double y, double th = 0) 
    { 
		setX(x); 
		setY(y); 
		setTh(th); 
	}
	
	virtual void setPose(SosPose& otherPose)
    {
		setX(otherPose.getX());
		setY(otherPose.getY());
		setTh(otherPose.getTh());
    }
	
	void setX(double x) 
	{ 
		_x = x; 
	}
	
	void setY(double y) 
	{ 
		_y = y; 
	}
	
	void setTh(double th) 
	{ 
		_th = fixAngle(th); 
	}
	
	inline double getX(void) const 
	{ 
		return _x; 
	}
	
	inline double getY(void) const 
	{ 
		return _y; 
	}
	
	inline double getTh(void) const 
	{ 
		return _th; 
	}
	
	void getPose(double *x, double *y, double *th = NULL) 
    { 
		if (x != 0) 
		{
			*x = _x;
		}
		if (y != 0) 
		{
			*y = _y; 
		}
		if (th != 0) 
		{
			*th = _th; 
		}
    }
	
	virtual double findDistanceTo(SosPose otherPose)
    {
		return sqrt((getX() - otherPose.getX()) * (getX() - otherPose.getX()) +
			(getY() - otherPose.getY()) * (getY() - otherPose.getY()));
    }

	virtual double findAngleTo(SosPose otherPose)
    {
		return SosUtil::radToDeg(atan2(otherPose.getY() - getY(),
			otherPose.getX() - getX()));
    }

	double radToDeg(double rad)  
	{ 
		return rad * 180.0 / 3.1415927; 
	}

	double fixAngle(double angle) 
	{
		if (angle >= 360)
		{
			angle = angle - 360.0 * (double)((int)angle / 360);
		}
		if (angle < -360)
		{
			angle = angle + 360.0 * (double)((int)angle / -360);
		}
		if (angle <= -180)
		{
			angle = + 180.0 + (angle + 180.0);
		}
		return angle;
	}
	
protected:
	double _x;
	double _y;
	double _th;
};

#define BUCKET_SIZE 5
class PoseRec
{
    friend ostream &operator<<(ostream&, PoseRec);
    friend istream &operator>>(istream&, PoseRec);
    
public:
    PoseRec();
	PoseRec(const PoseRec&);//copy constructor
    PoseRec(int x);
    const PoseRec &operator=(const PoseRec&);
    bool operator==(const PoseRec&) const;
    bool operator!=(const PoseRec&) const;
    
    bool wasSet[BUCKET_SIZE];
};

class LineObj
{ 
    friend ostream &operator<<(ostream&, const LineObj&);
	
public: 
    LineObj();
    void reset();
    
    void setLine(double x1, double y1, double x2, double y2);
    void setLineByAngle(double midPointX, double midPointY, double angle, double length);
	void setLineAsBisector(double x1, double y1, double x2,double y2, double length);
	
    void getMidPoint(double &x, double& y);
    bool getIntersection(LineObj, double &x, double &y, bool linesAreInfinite = false);
    double getLength();
	bool getSlope(double &);
	bool getYIntercept(double &);
	
    double getPerpDistTo(double X, double Y);//gets the perpendicular distance from (X,Y) to the line
    double getAngleBetween(LineObj);
    void translate(double X, double Y, double angle = 0);
	
	double getX1();
	double getY1();
	double getX2();
	double getY2();
	bool getYfromX(double x, double &y);
	bool getXfromY(double y, double &x);
	void getPoints(double &x1Point,double &y1Point, double &x2Point, double &y2Point);
	double getAngle();    
	
private:	
	double x1, y1, x2, y2;
	double angle;
	double slope;
	
	
}; 

class LineNode
{
public:
	LineNode(){next = 0; prev = 0;}
	LineObj myLine;	
	LineNode* next; 
	LineNode* prev;
};

ostream &operator<<(ostream &out, const LineObj& outLine);

#endif 