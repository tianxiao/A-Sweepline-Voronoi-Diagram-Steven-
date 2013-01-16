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

#ifndef DRAWABLE_OBJECT_H
#define DRAWABLE_OBJECT_H

#ifndef _WIN32_WINNT
	#define _WIN32_WINNT	0x0500
#endif

#include <windows.h>
#include <assert.h>

//if you use the XIMAGE library for manipulating images, define USE_CXIMAGE as 1, otherwise 0
#define USE_CXIMAGE 1

#if USE_CXIMAGE
#include "ximage.h"
#endif

#include "../sosutil/SosUtil.h"
#include "../logger/Logger.h"

#define SELECTED_BOX_WIDTH 5


class DrawableObject
{
public:
	DrawableObject(){_colour = RGB(0,0,0);}

//if we're using Windows, the draw function should be passed a HDC device handle to the window
#ifdef WIN32
	virtual void draw(HDC hdc) = 0;
	virtual void drawOpposite(HDC hdc){SetROP2(hdc, R2_NOT); draw(hdc);}
#endif

//if you have access to the CXImage library, you can use a draw function to draw to an image
#if USE_CXIMAGE
	virtual void draw(CxImage* image) = 0;
#endif
	
	virtual void setPoints(PointXYLong,PointXYLong) = 0;
	virtual void setPoints(long x1, long y1, long x2, long y2)=0;
	virtual void setColour(COLORREF colour) = 0;
			void setViewArea(PointXYLong pt){_viewArea = pt;}
			void getPoints(PointXYLong& pt1,PointXYLong& pt2){pt1 = _pt1; pt2 = _pt2;}
	virtual	void snapToOn(){};
	virtual	void snapToOff(){};			

	bool operator==(const DrawableObject& node) const{return (node._pt1==_pt1&&node._pt2==_pt2)||(node._pt1==_pt2&&node._pt2==_pt1);}
	bool operator!=(const DrawableObject& node) const{return !((node._pt1==_pt1&&node._pt2==_pt2)||(node._pt1==_pt2&&node._pt2==_pt1));}
	bool operator<(const DrawableObject& node) const{return (node._pt1 > _pt1)&&(node._pt2 > _pt2);}
	bool operator>(const DrawableObject& node) const{return (node._pt1 < _pt1)&&(node._pt2 < _pt2);}

protected:
	PointXYLong		_pt1, _pt2;
	COLORREF		_colour;
	PointXYLong		_viewArea;	
	
};


//this class only draws horizontal lines - very limited, but very lightwight
class DrawableScanLine : public DrawableObject
{
public:
	DrawableScanLine()
	{
		GET_FILE_LOG
		_pt1 = _pt2 = 0;
		_colour = RGB(0,0,0);
		_pen = ::CreatePen(PS_SOLID,1, _colour);
	}
	DrawableScanLine(long x1, long y1, long x2, long y2, COLORREF colour = 0)
	{
		GET_FILE_LOG
		_pt1.x = x1;
		_pt1.y = y1;
		_pt2.x = x2;
		_pt2.y = y2;
		_colour = colour;
		_pen = ::CreatePen(PS_SOLID,1, _colour);
	}
	

	DrawableScanLine(PointXYLong point1, PointXYLong point2)
	{
		GET_FILE_LOG
		_pt1 = point1;
		_pt2 = point2;
		_colour = RGB(0,0,0);
		_pen = ::CreatePen(PS_SOLID,1, _colour);
	}

	~DrawableScanLine()
	{
		DeleteObject (_pen);
	}

	//pt2.y is ignored, only the first y point is used
	virtual void setPoints(long x1, long y1, long x2, long y2)
	{
		//SosUtil::ensureSmaller(x1,x2);
		//SosUtil::ensureSmaller(y1,y2);
		_pt1.x = x1;
		_pt2.x = x2;
		_pt1.y = y1;
		_pt2.y = y2;		
	}

	//pt2.y is ignored, only the first y point is used
	virtual void setPoints(PointXYLong p1, PointXYLong p2){_pt1 = p1; _pt2 = p2;}
	

#ifdef WIN32
	void draw(HDC hdc)
	{
		::SelectObject(hdc,_pen);

		long x1 = _pt1.x,x2 = _pt2.x,y1 = _pt1.y;

		//only compare the x's, because the y's are the same
		if(x1 == x2 )
		{
			SetPixel(hdc,x1,y1,_colour);
			return;
		}

		SosUtil::ensureSmaller(x1,x2);

		if((y1 > _viewArea.y ) || (y1 < 0 ) || (x2 < 0 ) || (x1 > _viewArea.x ))
		{
			
			return; //if the line is completely outside the view area, don't draw it
		}		
		
		//clip the line within the view area
		if(x1 < 0)
			x1 = 0;

		if(x2 > _viewArea.x)
			x2 = _viewArea.x;		

		MoveToEx(hdc, x1, y1, (LPPOINT) NULL);
		if(LineTo(hdc, x2 ,y1) ==0)//y2 is ignored, only use y1
		{
			LOG<<"Line from ("<<x1<<","<<y1<<") -> ("<<x2<<","<<y1<<") was not drawn";
		}
	}
#endif

#if USE_CXIMAGE
	virtual void draw(CxImage* image)
	{
		//shift the X's 1 to the left to compensate for the fact that 
		//the Image starts at (0,0), not (1,1) like the screen
		long x1 = _pt1.x,x2 = _pt2.x,y1 = _pt1.y;

		//only compare the x's, because the y's are the same
		if(x1 == x2 )
		{
			image->SetPixelColor(x1,y1,_colour);

		//	SetPixel(hdc,x1,y1,_colour);
			return;
		}

		SosUtil::ensureSmaller(x1,x2);

		if((y1 > _viewArea.y ) || (y1 < 0 ) || (x2 < 0 ) || (x1 > _viewArea.x ))
		{
			
			return; //if the line is completely outside the view area, don't draw it
		}		
		
		//clip the line within the view area
		if(x1 < 0)
			x1 = 0;

		if(x2 > _viewArea.x)
			x2 = _viewArea.x;	
		
		image->DrawLine(x1, x2, y1, y1, _colour);
	}
#endif

	void setColour(COLORREF colour)
	{
		_colour = colour;
		DeleteObject (_pen);
		_pen = ::CreatePen(PS_SOLID,1, colour);
	}

	void	snapToOn()				{}//snapTo has no effect on this line - it's always horizontal
	void	snapToOff()				{}
	

protected:
	HPEN _pen;	
	DEF_LOG

};


class DrawableLine : public DrawableObject
{
public:
	DrawableLine()
	{
		GET_FILE_LOG
		_pt1 = _pt2 = 0;
		_colour = RGB(0,0,0);
		_pen = ::CreatePen(PS_SOLID,1, _colour);
	}
	DrawableLine(long x1, long y1, long x2, long y2, COLORREF colour = 0)
	{
		GET_FILE_LOG
		_pt1.x = x1;
		_pt1.y = y1;
		_pt2.x = x2;
		_pt2.y = y2;
		_colour = colour;
		_pen = ::CreatePen(PS_SOLID,1, _colour);
	}

	DrawableLine(PointXYLong point1, PointXYLong point2)
	{
		GET_FILE_LOG
		_pt1 = point1;
		_pt2 = point2;
		_colour = RGB(0,0,0);
		_pen = ::CreatePen(PS_SOLID,1, _colour);
	}

	~DrawableLine()
	{
		DeleteObject (_pen);
	}

	virtual void setPoints(long x1, long y1, long x2, long y2)
	{
		//SosUtil::ensureSmaller(x1,x2);
		//SosUtil::ensureSmaller(y1,y2);
		_pt1.x = x1;
		_pt2.x = x2;
		_pt1.y = y1;
		_pt2.y = y2;		
	}
	virtual void setPoints(PointXY pt1,PointXY pt2)
	{
		setPoints((long)pt1.x,(long)pt1.y,(long)pt2.x,(long)pt2.y);
	}

	virtual void setPoints(PointXYLong p1, PointXYLong p2){_pt1 = p1; _pt2 = p2;}
	void setPoint1(PointXYLong point){_pt1 = point;}
	void setPoint2(PointXYLong point){_pt2 = point;}
	void setPoint1(long x, long y){_pt1.x = x;_pt1.y = y;}
	void setPoint2(long x, long y){_pt2.x = x;_pt2.y = y;}

#ifdef WIN32
	void draw(HDC hdc)
	{
		::SelectObject(hdc,_pen);

		long x1 = _pt1.x,x2 = _pt2.x,y1 = _pt1.y,y2 = _pt2.y;
		double slope = 0, yIntercept = 0;

		//if the line is just a point, draw that pixel and finish
		if(x1 == x2 && y1 == y2)
		{
			SetPixel(hdc,x1,y1,_colour);
			return;
		}


		//if one or more of the points is outside the view area, it will require more processing
		//if(x1 > _viewArea.x || x2 > _viewArea.x ||y1 > _viewArea.y || y2 > _viewArea.y)
		if(!SosUtil::between(x1,0,_viewArea.x) || !SosUtil::between(x2,0,_viewArea.x) ||
			!SosUtil::between(y1,0,_viewArea.y) || !SosUtil::between(y2,0,_viewArea.y))
		{
			if((y1 > _viewArea.y && y2 > _viewArea.y) || (y1 < 0 && y2 < 0) ||
				(x1 < 0 && x2 < 0) || (x1 > _viewArea.x && x2 > _viewArea.x))
			{
			//	LOG<<"DrawableLine returing without drawing 1";
			//	LOG<<"x1 = "<<x1<<", x2 = "<<x2<<", y1 = "<<y1<<", y2 = "<<y2<<", view.x = "<<_viewArea.x<<", view.y = "<<_viewArea.y;
				
				return; //if the line is completely outside the view area, don't draw it
			}	
			//if y = mx +c, then x = (y - c)/m
	
			if(x1 == x2)
			{
				slope = 0;
			}
			else 
			{
				slope = double(y2 - y1)/double(x2 - x1);
				yIntercept = y1 - (slope * x1); //if y=mx+c, then c = y - mx
			}
			
			if(x1 > x2)//make sure pt 1 is to the left
			{
				SosUtil::swap(x1,x2);
				SosUtil::swap(y1,y2);
			}
			
			if(x2 > _viewArea.x)
			{
				x2 = _viewArea.x ;
				y2 = (slope != 0)? (long)((slope * x2) + yIntercept) : y2;
			}

			if(x1 < 0)
			{
				x1 = 1;
				y1 = (slope != 0)? (long)((slope * x1) + yIntercept) : y1;
			}
			
			if(y1 > y2)
			{
				SosUtil::swap(x1,x2);
				SosUtil::swap(y1,y2);
			}
			
			if(y2 > _viewArea.y)
			{
				y2 = _viewArea.y;
				x2 = (slope != 0)? (long)((y2 - yIntercept)/slope) : x2;
			}

			if(y1 < 0)
			{
				y1 = 1;
				x1 = (slope != 0)? (long)((y1 - yIntercept)/slope) : x1;
			}
			
			if((y1 > _viewArea.y || y2 > _viewArea.y) || (y1 < 0 && y2 < 0) ||
				(x1 < 0 && x2 < 0) || (x1 > _viewArea.x || x2 > _viewArea.x))
			{
			//	LOG<<"DrawableLine returing without drawing 2";
			//	LOG<<"x1 = "<<x1<<", x2 = "<<x2<<", y1 = "<<y1<<", y2 = "<<y2<<", view.x = "<<_viewArea.x<<", view.y = "<<_viewArea.y;
				return; //if the line is completely outside the view area, don't draw it
			}	
		}
		

		MoveToEx(hdc, x1, y1, (LPPOINT) NULL);
		LineTo(hdc, x2 ,y2);
	}

#endif

#if USE_CXIMAGE
	virtual void draw(CxImage* image)
	{
		long x1 = _pt1.x,x2 = _pt2.x,y1 = _pt1.y,y2 = _pt2.y;
		double slope = 0, yIntercept = 0;

		if(x1 == x2 && y1 == y2)
		{
			image->SetPixelColor(x1,y1,_colour);
			return;
		}


		//if one or more of the points is outside the view are, it will require more processing
		if(x1 > _viewArea.x || x2 > _viewArea.x ||y1 > _viewArea.y || y2 > _viewArea.y)
		{
			if((y1 > _viewArea.y && y2 > _viewArea.y) || (y1 < 0 && y2 < 0) ||
				(x1 < 0 && x2 < 0) || (x1 > _viewArea.x && x2 > _viewArea.x))
			{
				return; //if the line is completely outside the view area, don't draw it
			}	
			//if y = mx +c, then x = (y - c)/m
		//	if(y1 != y2)
			//	{
			if(x1 == x2)
			{
				slope = 0;
			}
			else 
			{
				slope = double(y2 - y1)/double(x2 - x1);
				yIntercept = y1 - (slope * x1); //if y=mx+c, then c = y - mx
			}
			
			if(x1 > x2)//make sure pt 1 is to the left
			{
				SosUtil::swap(x1,x2);
				SosUtil::swap(y1,y2);
			}
			
			if(x2 >= _viewArea.x)
			{
				x2 = _viewArea.x -1;
				y2 = (slope != 0)? (long)((slope * x2) + yIntercept) : y2;
			}
			
			if(y1 > y2)
			{
				SosUtil::swap(x1,x2);
				SosUtil::swap(y1,y2);
			}
			
			if(y2 >= _viewArea.y)
			{
				y2 = _viewArea.y-1;
				x2 = (slope != 0)? (long)((y2 - yIntercept)/slope) : x2;
			}
			
			if((y1 >= _viewArea.y || y2 >= _viewArea.y) || (y1 < 0 && y2 < 0) ||
				(x1 < 0 && x2 < 0) || (x1 >= _viewArea.x || x2 >= _viewArea.x))
			{
			//	LOG<<"DrawableLine returing without drawing";
			//	LOG<<"x1 = "<<x1<<", x2 = "<<x2<<", y1 = "<<y1<<", y2 = "<<y2<<", view.x = "<<_viewArea.x<<", view.y = "<<_viewArea.y;
				return; //if the line is completely outside the view area, don't draw it
			}	
		}
		

		image->DrawLine(x1, x2, y1, y2, _colour);
	}
#endif

	void setColour(COLORREF colour)
	{
		_colour = colour;
		DeleteObject (_pen);
		_pen = ::CreatePen(PS_SOLID,1, colour);
	}

	void	snapToOn()				{snapTo = true;}
	void	snapToOff()				{snapTo = false;}
	

protected:
	HPEN _pen;	
	bool	snapTo;
	DEF_LOG
};



class DrawablePixel : public DrawableObject
{
public:
	DrawablePixel()
	{

	}
	DrawablePixel(PointXYLong pt, COLORREF colour = 0)
	{
		_pt1 = pt;
		_colour = colour;
	}
	void setPoints(PointXYLong p1, PointXYLong p2){_pt1 = p1;}//point 2 is discarded

#ifdef WIN32
	void draw(HDC hdc)
	{
		SetPixel(hdc,_pt1.x,_pt1.y,_colour);
	}
#endif

#if USE_CXIMAGE
	virtual void draw(CxImage* image)
	{
		image->SetPixelColor(_pt1.x,_pt1.y,_colour);
	}
#endif

	void setColour(COLORREF colour)
	{
		_colour = colour;
	}

	void setPoints(long x1, long y1, long x2, long y2)
	{		
		_pt1.x = x1;		
		_pt1.y = y1;		
	}

	void	snapToOn()				{}
	void	snapToOff()				{}


private:

};

class DrawableRectangleFilled : public DrawableObject
{
public:
	DrawableRectangleFilled()
	{
		_colour = 0;
		_pt1 = _pt2 = 0;
	}
	DrawableRectangleFilled(PointXYLong point1, PointXYLong point2, COLORREF colour)
	{
		init(point1,point2);
		_colour = colour;
	}

	void setPoints(PointXYLong point1, PointXYLong point2)
	{
		init(point1,point2);
	}

	void setPoints(long x1, long y1, long x2, long y2)
	{
		SosUtil::ensureSmaller(x1,x2);
		SosUtil::ensureSmaller(y1,y2);
		_pt1.x = x1;
		_pt2.x = x2;
		_pt1.y = y1;
		_pt2.y = y2;		
	}
	virtual void setPoints(PointXY pt1,PointXY pt2)
	{
		setPoints((long)pt1.x,(long)pt1.y,(long)pt2.x,(long)pt2.y);
	}

	void setColour(COLORREF colour)
	{
		_colour = colour;
	}

#ifdef WIN32
	virtual void draw(HDC hdc)
	{
		::SetBkColor(hdc, _colour);

		long x1 = _pt1.x, x2 = _pt2.x,y1 = _pt1.y, y2 = _pt2.y;

		SosUtil::ensureSmaller(x1,x2);
		SosUtil::ensureSmaller(y2,y1);

		if(x1 > _viewArea.x || y2 > _viewArea.y)
			return;

		if(x2 > _viewArea.x)
			x2 = _viewArea.x;

		if(y1 > _viewArea.y)
			y1 = _viewArea.y;


		RECT rect = { x1, y1, x2,y2 };
		::ExtTextOut(hdc, 0, 0, ETO_OPAQUE, &rect, NULL, 0, NULL);
	}
#endif

#if USE_CXIMAGE

	virtual void draw(CxImage* image)
	{
		DrawableLine line;
		line.setColour(_colour);
		line.setViewArea(_viewArea);
		
		long minY = SosUtil::minVal(_pt1.y,_pt2.y);
		long maxY = SosUtil::maxVal(_pt1.y,_pt2.y);

		for(long y = minY; y<= maxY; y++)
		{
			line.setPoints(_pt1.x,y,_pt2.x,y);
			line.draw(image);
		}
	}
#endif

	void	snapToOn()				{snapTo = true;}
	void	snapToOff()				{snapTo = false;}

private:	

	inline void init(PointXYLong point1, PointXYLong point2)
	{
		SosUtil::ensureSmaller(point1.x,point2.x);
		SosUtil::ensureSmaller(point1.y,point2.y);

		_pt1 = point1;
		_pt2 = point2;		
	}
	bool	snapTo;
	

};


class DrawableRectangle : public DrawableObject
{
public:
	DrawableRectangle()
	{
	//	GET_FILE_LOG
		LOGGING_OFF
		_pt1 = _pt2 = 0;
		_colour = RGB(0,0,0);
		_pen = ::CreatePen(PS_SOLID,1, _colour);
	}

	DrawableRectangle(PointXYLong point1, PointXYLong point2)
	{
		GET_FILE_LOG
		init(point1,point2);
		_colour = RGB(0,0,0);
		_pen = ::CreatePen(PS_SOLID,1, _colour);
	}

	~DrawableRectangle()
	{
		::DeleteObject (_pen);
	}	

	void setPoints(PointXYLong point1, PointXYLong point2)
	{
		init(point1,point2);
	}
	virtual void setPoints(PointXY pt1,PointXY pt2)
	{
		setPoints((long)pt1.x,(long)pt1.y,(long)pt2.x,(long)pt2.y);
	}

#ifdef WIN32
	virtual void draw(HDC hdc)
	{	
		LOG<<"DrawableRectangle::draw()";
		::SelectObject(hdc,_pen);

		long x1 = _pt1.x, x2 = _pt2.x,y1 = _pt1.y,y2 = _pt2.y;
		SosUtil::ensureSmaller(x1,x2);
		SosUtil::ensureSmaller(y2,y1);

		if(x1 >= _viewArea.x || y2 >= _viewArea.y)
		{
			LOG<<"Not drawing Rectangle because _pt1.x = "<<x1<<", _pt2.y = "<<y1<<" and the view area is ("<<_viewArea.x<<","<<_viewArea.y<<")";
			return;
		}
		
		LOG<<"Rectangle is in view area";

		if(x2 > _viewArea.x)
			x2 = _viewArea.x;

		if(y1 > _viewArea.y)
			y1 = _viewArea.y;

		if(y1 < _viewArea.y)
		{
			MoveToEx(hdc, x1, y1, (LPPOINT) NULL);//bottom line
			LineTo(hdc, x2 ,y1);
		}
		else
		{
			y1 = _viewArea.y - 1;
		}

		MoveToEx(hdc, x1, y2, (LPPOINT) NULL);//top line
		LineTo(hdc, x2 ,y2);

		MoveToEx(hdc, x1, y1-1, (LPPOINT) NULL);//left line
		LineTo(hdc, x1 ,y2);

		if(x2 < _viewArea.x)
		{
			MoveToEx(hdc, x2, y1, (LPPOINT) NULL);//right line
			LineTo(hdc, x2 ,y2-1);
		}		
	}
#endif

#if USE_CXIMAGE
	virtual void draw(CxImage* image)
	{		
		long x1 = _pt1.x, x2 = _pt2.x,y1 = _pt1.y,y2 = _pt2.y;
		SosUtil::ensureSmaller(x1,x2);
		SosUtil::ensureSmaller(y2,y1);

		if(x1 >= _viewArea.x || y2 >= _viewArea.y)
		{
			LOG<<"Not drawing Rectangle because _pt1.x = "<<x1<<", _pt2.y = "<<y1<<" and the view area is ("<<_viewArea.x<<","<<_viewArea.y<<")";
			return;
		}
		
		LOG<<"Rectangle is in view area";

		if(x2 > _viewArea.x)
			x2 = _viewArea.x;

		if(y1 > _viewArea.y)
			y1 = _viewArea.y;

		if(y1 < _viewArea.y)
		{
			image->DrawLine(x1, x2, y1, y1, _colour);//bottom line
		}
		else
		{
			y1 = _viewArea.y - 1;
		}

		image->DrawLine(x1, x2, y2, y2, _colour);//top line

		image->DrawLine(x1, x1, y1-1, y2, _colour);//left line

		if(x2 < _viewArea.x)
		{
			image->DrawLine(x2, x2, y1, y2-1, _colour);//right line
		}
	}
#endif

	void setPoints(long x1, long y1, long x2, long y2)
	{
		SosUtil::ensureSmaller(x1,x2);
		SosUtil::ensureSmaller(y1,y2);
		_pt1.x = x1;
		_pt2.x = x2;
		_pt1.y = y1;
		_pt2.y = y2;		
	}

	void setColour(COLORREF colour)
	{
		_colour = colour;
		::DeleteObject (_pen);
		_pen = ::CreatePen(PS_SOLID,1, _colour);
	}

	void	snapToOn()				{snapTo = true;}
	void	snapToOff()				{snapTo = false;}


protected:

	inline void init(PointXYLong point1, PointXYLong point2)
	{
		SosUtil::ensureSmaller(point1.x,point2.x);
		SosUtil::ensureSmaller(point1.y,point2.y);

		_pt1 = point1;
		_pt2 = point2;		
	}

	HPEN _pen;
	bool	snapTo;
	DEF_LOG
};



//treat a circle just like a rectangle
class DrawableCircle : public DrawableRectangle
{
public:
	DrawableCircle()
	{
	}

	void setRadiusAndCentre(const PointXYLong& centre, long radius)
	{
		setPoints(centre.x - radius, centre.y + radius, centre.x + radius, centre.y - radius);
	}	
#ifdef WIN32
	virtual void draw(HDC hdc)
	{
		draw(hdc,false);
	}

	virtual void draw(HDC hdc, bool drawOpposite)
	{	
		LOG<<"DrawableCircle::draw()";
		::SelectObject(hdc,_pen);
		//::SetBkColor(hdc, RGB(255,255,255));
		::SelectObject(hdc,::GetStockObject(NULL_BRUSH));

		if(drawOpposite)
		{
			SetROP2(hdc, R2_NOT); 
		}
		else
		{
			SetROP2(hdc, R2_COPYPEN); 
		}

		long x1 = _pt1.x, x2 = _pt2.x,y1 = _pt1.y,y2 = _pt2.y;
		SosUtil::ensureSmaller(x1,x2);
		SosUtil::ensureSmaller(y2,y1);

		if(x2 >= _viewArea.x || y1 >= _viewArea.y || x1 < 0 || y2 < 0)
		{
			LOG<<"Not drawing Circle because _pt1.x = "<<x1<<", _pt2.y = "<<y1<<" and the view area is ("<<_viewArea.x<<","<<_viewArea.y<<")";
			return;
		}
		
		Ellipse(hdc,x1,y1,x2,y2);			
	}
#endif

#if USE_CXIMAGE
	void draw(CxImage* image){}//do not draw onto a picture
#endif

private:
	float _angle;

};

class DrawableRobot : public DrawableCircle
{
public:
	DrawableRobot()
	{
		GET_FILE_LOG
		_angle = 0;
		_isVisible = true;
	}

	void setOrientation(float angle)
	{
		_angle = angle;		
	}

	float getOrientation()
	{
		return _angle;
	}

	void setVisible(bool val)
	{
		_isVisible = val;
	}

#ifdef WIN32
	virtual void draw(HDC hdc)
	{	
		draw(hdc,false);
	}

	virtual void draw(HDC hdc, bool drawingOpposite)
	{	
		LOG<<"In DrawableRobot::draw(HDC)"<<endl;
		if(!_isVisible)
		{
			return;
		}
		
		::SelectObject(hdc,_pen);

		if(drawingOpposite)
		{
			//::SelectObject(hdc,::GetStockObject(NULL_BRUSH));
			SetROP2(hdc, R2_NOT); 
		}
		else
		{
			//::SelectObject(hdc,::GetStockObject(WHITE_BRUSH));
			SetROP2(hdc, R2_COPYPEN); 
		}

		long x1 = _pt1.x, x2 = _pt2.x,y1 = _pt1.y,y2 = _pt2.y;
		SosUtil::ensureSmaller(x1,x2);
		SosUtil::ensureSmaller(y2,y1);

		LOG<<"Points are ("<<x1<<","<<y1<<") and ("<<x2<<","<<y2<<")";

		if(x2 >= _viewArea.x || y1 >= _viewArea.y || x1 < 0 || y2 < 0)
		{
			LOG<<"Not drawing Robot because _pt1.x = "<<x1<<", _pt2.y = "<<y1<<" and the view area is ("<<_viewArea.x<<","<<_viewArea.y<<")";
			return;
		}

		PointXYLong centre(x1 + (x2 - x1)/2,y2+(y1 - y2)/2);

		double yDbl = 0;//(double)centre.y;
		double xDbl = 0;//

		xDbl = (double)(x2 - x1)/2;
		LOG<<"Point before the rotate of "<<_angle<<" degrees is ("<<xDbl<<","<<yDbl<<")"; 
		SosUtil::pointRotate(&xDbl,&yDbl,_angle);
		LOG<<"Point after the rotate is ("<<xDbl<<","<<yDbl<<")"; 

		xDbl += centre.x;
		yDbl += centre.y;

		::MoveToEx(hdc,centre.x,centre.y,(LPPOINT) NULL);
		::LineTo(hdc,(long)xDbl,(long)yDbl);

		DrawableCircle::draw(hdc, drawingOpposite);

		LOG<<"DrawableRobot::draw() Drew line from centre ("<<centre.x<<","<<centre.y<<") to ("<<xDbl<<","<<yDbl<<"), _angle = "<<_angle<<endl;
	}
#endif

	float getAngle()
	{
		return _angle;
	}

protected:
	float _angle;
	bool _isVisible;
};



#endif

