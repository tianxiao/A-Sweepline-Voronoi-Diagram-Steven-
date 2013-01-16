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

#include <stdlib.h>



#include "SosUtil.h"
#include <time.h>
#include <windows.h>
#include <ctype.h>


bool SosUtil::_randInitialised = false;
bool SosUtil::_initialised = false;
//double SosUtil::cachedSinVals[36000] = {0};
//double SosUtil::cachedCosVals[36000] = {0};

void SosUtil::init()
{
	if(!_initialised)
	{
		_initialised = true;
		srand( (unsigned)time( NULL ) );
	}
}

SosUtil::SosUtil()
{

}

double SosUtil::getDist(double x1, double y1, double x2, double y2)
{
	return sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));
} 

double SosUtil::getDist(long x1, long y1, long x2, long y2)
{
	return sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));
} 

double SosUtil::getDist(PointXY pt1, PointXY pt2)
{
	return (float)getDist((double)pt1.x,(double)pt1.y,(double)pt2.x,(double)pt2.y);
}

double SosUtil::getDist(PointXYLong pt1, PointXYLong pt2)
{
	return getDist((long)pt1.x,(long)pt1.y,(long)pt2.x,(long)pt2.y);
}
	

double SosUtil::minVal(double num1, double num2)
{
	if(num1 > num2)
		return num2;
	else 
		return num1;
}

double SosUtil::maxVal(double num1, double num2)
{
	if(num1 < num2)
		return num2;
	else 
		return num1;
}

float SosUtil::minVal(float num1, float num2)
{
	if(num1 > num2)
		return num2;
	else 
		return num1;
}

float SosUtil::maxVal(float num1, float num2)
{
	if(num1 < num2)
		return num2;
	else 
		return num1;
}

long SosUtil::minVal(long num1, long num2)
{
	if(num1 > num2)
		return num2;
	else 
		return num1;
}

long SosUtil::maxVal(long num1, long num2)
{
	if(num1 < num2)
		return num2;
	else 
		return num1;
}


long SosUtil::minVal(long num1, long num2, long num3)
{
	if(num1 < num2 )
	{
		if(num1 < num3)
			return num1;

		return num3;
	}

	if(num2 < num3)
		return num2;

	return num3;

}
long SosUtil::maxVal(long num1, long num2, long num3)
{
	if(num1 > num2 )
	{
		if(num1 > num3)
			return num1;

		return num3;
	}

	if(num2 > num3)
		return num2;

	return num3;
}


void SosUtil::swap(long& num1, long& num2)
{
	long temp = 0;
	temp = num1;
	num1 = num2;
	num2 = temp;
}

void SosUtil::swap(float& num1, float& num2)
{
	float temp = 0;
	temp = num1;
	num1 = num2;
	num2 = temp;
}

void SosUtil::swap(double& num1, double& num2)
{
	double temp = 0;
	temp = num1;
	num1 = num2;
	num2 = temp;
}

bool SosUtil::ensureSmaller(long& num1, long& num2)
{
	if(num2 < num1)
	{
		swap(num1,num2);
		return true;
	}
	return false;
}

bool SosUtil::ensureSmaller(double& num1, double& num2)
{
	if(num2 < num1)
	{
		swap(num1,num2);
		return true;
	}
	return false;
}

bool SosUtil::ensureSmaller(float& num1, float& num2)
{
	if(num2 < num1)
	{
		swap(num1,num2);
		return true;
	}
	return false;
}



double SosUtil::fabs(double num)
{
	if(num >= 0)
		return num;

	return num * -1;
}

float SosUtil::fabs(float num)
{
	if(num >= 0)
		return num;

	return num * -1;
}

long SosUtil::fabs(long num)
{
	if(num >= 0)
		return num;

	return num * -1;
}


long SosUtil::getRandomNumber(long min, long max)
{	
	if(!_randInitialised)
	{
		srand( (unsigned)time( NULL ) );
		_randInitialised = true;
	}
	max ++;
	if(min == max ) 
		return min;
	
	SosUtil::ensureSmaller(min,max);
	
	long diff = max - min ;
	
	long ret = (rand() % diff) + min;


	
	return ret;
}

double SosUtil::getRandomNumber(double min, double max, int precision)
{
	if(!_randInitialised)
	{
		srand( (unsigned)time( NULL ) );
		_randInitialised = true;
	}

	int precisionCount = precision;
	double ret = 0;
	long minLong = 0, maxLong = 0;
	long div = 1;

	while(precisionCount > 0)
	{
		min *= 10;
		max *= 10;
		div *= 10;
		precisionCount --;
	}

	minLong = (long)min;
	maxLong = (long)max;

	ret = getRandomNumber(minLong,maxLong);

	ret /= div;
	return ret;
}

bool SosUtil::getRandomBool()
{
	long ret = getRandomNumber(0,99);

	if(ret >= 50)
		return true;
	else
		return false;
}

void SosUtil::generateRandomDistribution(float* arr,int size, float average)
{
	float totalSum = size * average;
	float initSum = 0;

	int i = 0;
	for(i = 0; i< size; i++)
	{
		arr[i] = (float)getRandomNumber(0,1,5);
		initSum += arr[i];
	}

	float mul = totalSum / initSum;

	for(i = 0; i< size; i++)
	{
		arr[i] *= mul;
	}
}

void SosUtil::generateRandomDistributionWithMin(float* arr,int size, float average, float min)
{
	float totalSum = size * average;
	float initSum = 0;

	int i = 0;
	for(i = 0; i< size; i++)
	{
		arr[i] = (float)getRandomNumber(0,1,5);
		initSum += arr[i];
	}

	float mul = totalSum / initSum;
//	long numTooSmall = 0;
	
	for(i = 0; i < size; i++)
	{
		if(arr[i] * mul < min)
		{
			initSum -= arr[i];	
			arr[i] = min / mul;
			initSum += arr[i];
			//numTooSmall++;
		}
	}

	mul = totalSum / initSum;

	for(i = 0; i< size; i++)
	{
		if(arr[i] * mul < min)
		{
			arr[i] = min;
		}
		else
		{
			arr[i] *= mul;
		}
	}
}

void SosUtil::sleep(unsigned int ms)
{
#ifdef WIN32
	Sleep(ms);
	cout<<"Called sleep\n";
#endif // WIN32
#ifdef linux
	if (ms > 10)
		ms -= 10;
	usleep(ms * 1000);
#endif // linux
	
}
bool SosUtil::is_numeric(char * str) 
{
	if ( strspn ( str, "0123456789.+-eE" ) != strlen(str) ) 
		return ( false) ;
	else
		return ( true) ;
}

bool SosUtil::is_integer(char * str)
{
	if ( strspn ( str, "0123456789+-" ) != strlen(str) ) 
		return ( false) ;
	else
		return ( true) ;
}

bool SosUtil::endsWith(char* str1, char* ending)
{
	if(str1 == 0 || ending == 0)
		return false;

	int lengthEnding = strlen(ending);
	int lengthString = strlen(str1);

	if(lengthString < lengthEnding)
		return false;

	return (stricmp(str1 + (lengthString - lengthEnding),ending) == 0);


}

char* SosUtil::doubleToString(double val, int precision, char* displayString)
{
	int pos1=0,pos2=0;
	char* buf = _fcvt(val,precision,&pos1,&pos2);
	char buf2[50] = {0};

	int count = 0;
	if(pos1 < 0)
	{
		buf2[0] = '.';
		count++;
		while(pos1 < 0)
		{
			pos1++;
			buf2[count] = '0';
			count++;
		}
		strcpy(buf2+count,buf);
	}
	else if(pos1 > 0)
	{
		strncpy(buf2,buf,pos1);
		count = strlen(buf2);

		buf2[pos1]='.';
		
		strcpy(buf2 + pos1 +1,buf + pos1);

	}
	else
	{
		buf2[0]='.';
		strcpy(buf2+1,buf);
	}
	
	if(val < 0)
	{
		displayString[0] = '-';
		strcpy(displayString + 1,buf2);
	}
	else
	{
		
		strcpy(displayString,buf2);
	}
	return displayString;
}

char* SosUtil::floatToString(float val, int precision, char* displayString)
{
	int pos1=0,pos2=0;
	char* buf = _fcvt(val,precision,&pos1,&pos2);
	char buf2[50] = {0};

	int count = 0;
	if(pos1 < 0)
	{
		buf2[0] = '.';
		count++;
		while(pos1 < 0)
		{
			pos1++;
			buf2[count] = '0';
			count++;
		}
		strcpy(buf2+count,buf);
	}
	else if(pos1 > 0)
	{
		strncpy(buf2,buf,pos1);
		count = strlen(buf2);

		buf2[pos1]='.';
		
		strcpy(buf2 + pos1 +1,buf + pos1);

	}
	else
	{
		buf2[0]='.';
		strcpy(buf2+1,buf);
	}
	
	if(val < 0)
	{
		displayString[0] = '-';
		strcpy(displayString + 1,buf2);
	}
	else
	{
		
		strcpy(displayString,buf2);
	}
	return displayString;
   
}

bool SosUtil::readNum(FILE* file, int& num)
{
	if(file == 0) return false;

	char buf[256];
	fscanf(file,"%s",&buf);
	if(!is_numeric(buf))
		return false;

	double dblNum = atof(buf);
	num = (int)dblNum;
	return true;

}

bool SosUtil::readNum(FILE* file, long& num)
{
	if(file == 0) return false;

	char buf[256];
	fscanf(file,"%s",&buf);
	if(!is_numeric(buf))
		return false;

	double dblNum = atof(buf);
	num = (long)dblNum;
	return true;

}

bool SosUtil::readNum(FILE* file, float& num)
{
	if(file == 0) return false;

	char buf[256];
	fscanf(file,"%s",&buf);
	if(!is_numeric(buf))
		return false;

	num = (float)atof(buf);
	return true;

	
}


bool SosUtil::getCircleCentreFrom3Points(double x1, double y1, double x2, double y2, double x3, double y3,
		double &centreX, double &centreY)
{
	LineObj line1, line2;	
	
	double x = 0, y = 0;

	//set the length to 500, it doesn't matter what the length the lines are, when checking if they intersect
	//the fourth variable in the getIntrsection() is set to true, which means the lines are infinite in length
	line1.setLineAsBisector(x1, y1, x2, y2, 500);
	line2.setLineAsBisector(x2, y2, x3, y3, 500);

	//the the three points are in a straght line, a circle cannot be inscribed 
	if(!line1.getIntersection(line2,x, y, true))
	{
		return false;
	}
	else
	{
		centreX = x; 
		centreY = y;
		return true;
	}
}

void SosUtil::pointRotate(double *x, double *y, double th)
{
	double cs, sn, xt, yt;
	//SosUtil::cachedSinAndCos(th,sn,cs);
	cs = SosUtil::cos(th);
	sn = SosUtil::sin(th);
	xt = *x;  
	yt = *y;
	*x = cs*xt + sn*yt;
	*y = cs*yt - sn*xt;
} 

float SosUtil::setPrecision(float num, int pres)
{
	float mul = 1;
	for(int i = 0; i< pres; i++)
		mul *= 10;

	long temp = long(num * mul);
	return (float)temp / mul;

}

double SosUtil::radToDeg(double rad)  
{ 
	return rad * 180.0 / PI; 
}

double SosUtil::degToRad(double deg)  
{ 
	return deg * PI / 180.0; 
}

double SosUtil::atan2(double y, double x)  
{ 
	return (::atan2(y,x)) * 180.0 / PI;
	//return SosUtil::radToDeg(::atan2(y, x)); 
}

PointXY SosUtil::getMidPoint(LineXY line)
{
	PointXY point;
	point.x = (line.pt1.x  +line.pt2.x)/2;
    point.y = (line.pt1.y +line.pt2.y)/2;
	return point;
}

PointXYLong SosUtil::getMidPoint(LineXYLong line)
{
	PointXYLong point;
	point.x = (line.pt1.x  +line.pt2.x)/2;
    point.y = (line.pt1.y +line.pt2.y)/2;
	return point;
}

double SosUtil::cos(double angle) 
{ 
	return ::cos(SosUtil::degToRad(angle)); 
}
	
	
double SosUtil::sin(double angle)
{ 
	return ::sin(SosUtil::degToRad(angle)); 
}


void SosUtil::sinAndCos(double angle, double& sine, double& cosine)
{
	angle = angle * PI / 180.0;
	sine = ::sin(angle);
	cosine = ::cos(angle);
}

double SosUtil::roundDown(double num)
{
	if(num >= 0)
	{
		num = double(long(num));
	}
	else
	{
		num = double(long(num -1));
	}

	return num;
	 
}


double SosUtil::roundUp(double num)
{
	if(num > SosUtil::roundDown(num))
		return SosUtil::roundDown(num) + 1;
	else
		return num;
}


double SosUtil::fixAngle(double angle) 
{
	if (angle >= 360)
		angle = angle - 360.0 * (double)((int)angle / 360);
	if (angle < -360)
		angle = angle + 360.0 * (double)((int)angle / -360);
	if (angle <= -180)
		angle = + 180.0 + (angle + 180.0);
	//if (angle > 180)
	//	angle = - 180.0 + (angle - 180.0);
	return angle;
}

char* SosUtil::stripPath(char* str)
{
	if(str == 0)
		return "";

	long i = strlen(str) - 1;

	//search backwards - probably faster
	for(;i>= 0;i--)
	{
		if(str[i]=='\\')
			return str+i+1;
	}

	return str;
}

bool SosUtil::getPath(char* source, char* dest)
{
	if(source == 0 || dest == 0)
		return false;

	long i = strlen(source) - 1;

	//search backwards - probably faster
	for(;i>= 0;i--)
	{
		if(source[i]=='\\' || source[i] == '/')
		{
			strcpy(dest,source);
			dest[i] = '\0';
			return true;
		}
	}

	return false;
}

bool SosUtil::getFilenameAndPathNoExtension(char* source, char* dest)
{
	if(source == 0 || dest == 0)
		return false;

	strcpy(dest,source);

	
	long length = strlen(source);
	if(dest[length -1] == '\\' || dest[length -1] == '/')
	{
		dest[length -1] = '\0';
		length--;
	}
	long i = length - 1;

	
	//search backwards - probably faster
	for(;i>= 0;i--)
	{
		if(dest[i]=='\\' || dest[i] == '/')
		{			
			return true;
		}
		if(dest[i] == '.')
		{
			dest[i] = '\0';
			return true;
		}	
	}

	return false;
}

void SosUtil::getBaseFileName(char* source, char* dest)
{
	if(source == 0 || dest == 0)
		return;

	char* sourceNoPath = SosUtil::stripPath(source);

	strcpy(dest,sourceNoPath);

	long i = strlen(dest);

	//search backwards, and set the first '.' to zero
	for(; i >= 0; i--)
	{
		if(dest[i] == '.')
		{
			dest[i] = '\0';
			break;
		}
	}
}

int SosUtil::stringInArray(char* str, char** strArray, int arrSize)
{
	if(str == 0 || strlen(str) == 0 || strArray == 0 || arrSize < 1)
		return -1;

	for(int i = 0; i < arrSize; i++)
	{
		if(strcmp(str,strArray[i]) == 0)
			return i;
	}

	return -1;
}

int SosUtil::stringInArrayNoCase(char* str, char** strArray, int arrSize)
{
	if(str == 0 || strlen(str) == 0 || strArray == 0 || arrSize < 1)
		return -1;

	for(int i = 0; i < arrSize; i++)
	{
		if(stricmp(str,strArray[i]) == 0)
			return i;
	}

	return -1;
}

int SosUtil::firstInstance(const char* string, const char* key, bool ignoreCase)
{
	if(string == 0 || key == 0)
		return -1;

	long strlen2 = strlen(key), strlen1 = strlen(string);

	if(strlen2 > strlen1)
		return -1;

	int foundPos = -1;
	bool found = false;

	if(!ignoreCase)
	{
		for(int i = 0; i < strlen1 - strlen2 + 1 ; i++)
		{
			if(string[i] == key[0])
			{
				found = true;
				for(int j = 1; j< strlen2 ; j++)
				{
					if(key[j] != string[i + j])
					{
						found = false;
						break;
					}
					foundPos = i + j;
				}
				if(found)
					return i;
			}
		}
	}
	else
	{
		for(int i = 0; i < strlen1 - strlen2 + 1 ; i++)
		{
			if(tolower(string[i]) == tolower(key[0]))
			{
				found = true;
				for(int j = 1; j< strlen2 ; j++)
				{
					if(tolower(key[j]) != tolower(string[i + j]))
					{
						found = false;
						break;
					}
					foundPos = i + j;
				}
				if(found)
					return i;
			}
		}
	}
	return -1;
}


bool SosUtil::trim(const char* source, char* dest)
{
	bool trimmed = false;
	long len = (long)strlen(source);
	long startPos = 0, endPos = len - 1;

	for(startPos = 0; startPos < endPos; startPos++)
	{
		if(!(source[startPos] == ' ' || source[startPos] == '\n'
			|| source[startPos] == '\t'))
		{
			break;
		}
		trimmed = true;
	}

	for(; endPos >= startPos; endPos--)
	{
		if(!(source[endPos] == ' ' || source[endPos] == '\n'
			|| source[endPos] == '\t'))
		{
			break;
		}
		trimmed = true;
	}

	if(!trimmed)
		return false;

	strncpy(dest,source + startPos,endPos - startPos + 1);

	return true;
}

bool SosUtil::compareFiles(char* file1, char* file2)
{
	ifstream in1, in2;

	in1.open(file1);
	
	if(!in1.is_open())
		return false;
	
	in2.open(file2);

	if(!in2.is_open())
		return false;

	bool filesEqual = true;
	char ch1 = 0, ch2 = 0;

	in1>>ch1;
	in2>>ch2;
	if(ch1 != ch2 || in1.eof() != in2.eof())
	{
		filesEqual = false;
	}

	while(filesEqual && !in1.eof() && !in2.eof())
	{
		in1>>ch1;
		in2>>ch2;
		if(ch1 != ch2 || in1.eof() != in2.eof())
		{
			filesEqual = false;
		}	
	}

	in1.close();
	in2.close();

	return filesEqual;
}

int SosUtil::tokenise(const char* source, char** tokens, const char* delimiters, 
					  int numDelimiters, int firstTokenNum, int lastTokenNum)
{
	GET_FILE_LOG_GLOBAL

	if(source == 0 || tokens == 0 || delimiters == 0 || firstTokenNum > lastTokenNum)
	{
		return -1;
	}

	int count = 1, i = 0, j=0, pre = 0;
	bool found = false;
	bool inDelim = true;
	int tokenNum = 1;

	//first check if the first character is a delimiter.  If so, then move the string
	//pointer to the first non-delimiter character
	while(inDelim && source[i+1] != '\n' && source[i+1] != '\0')
	{
		found = false;
		for(j = 0; !found && j < numDelimiters; j++)
		{
			if(source[0] == delimiters[j])
				found = true;
		}

		if(found)
		{
			source++;
			pre++;
		}
		else 
			inDelim = false;
	}

	inDelim = false;

	int tokenPos = 0;

	for(i = 0; tokenNum <= lastTokenNum && source[i] != '\n' && source[i] != '\0' ; i++)
	{
	//	cout<<"source["<<i<<"] = "<<(int)source[i]<<" and \\n = "<<(int)'\n'<<" and \\0 = "<<(int)'\0'<<endl;
		found = false;
		for(j = 0; !found && j < numDelimiters; j++)
		{
			if(source[i] == delimiters[j])
				found = true;
		}
		
		if(!found)
		{
			if(inDelim)
			{
				tokenNum++;//go on to the next array
				tokenPos = 0;//reset the counter to point to the first place in the array
				inDelim = false;
			}

			if(tokenNum >= firstTokenNum && tokenNum <= lastTokenNum)
			{
				tokens[tokenNum - firstTokenNum][tokenPos] = source[i];
				tokens[tokenNum - firstTokenNum][tokenPos+1] = 0;
				tokenPos++;
			}
		}
		else
		{
			inDelim = true;
		}
	}

	if(tokenNum < lastTokenNum)
		return -1;

	//LOG<<"returning "<<i - 1 + pre<<", leaving the string:" <<source + (i - 1 + pre);
	return i - 1 + pre;

}

//"this is the original string"
//Skip the first 'tokenNum - 1' tokens in the 'source' string, and copy the next token into 
//the 'token' string.  the string MUST be terminated with a '\0' character
int SosUtil::getToken(const char* source, char* token, const char* delimiters, int numDelimiters, int tokenNum)
{
	int count = 1, i = 0, j=0, pre = 0;
	bool found = false;
	bool inDelim = true;

	//first check if the first character is a delimiter.  If so, then move the string
	//pointer to the first non-delimiter character
	while(inDelim && source[i+1] != '\n' && source[i+1] != '\0')
	{
		found = false;
		for(j = 0; !found && j < numDelimiters; j++)
		{
			if(source[0] == delimiters[j])
				found = true;
		}

		if(found)
		{
			source++;
			pre++;
		}
		else 
			inDelim = false;
	}

	inDelim = false;

	for(i = 0; count < tokenNum && source[i+1] != '\n' && source[i+1] != '\0' ; i++)
	{
		found = false;
		for(j = 0; !found && j < numDelimiters; j++)
		{
			if(source[i] == delimiters[j])
				found = true;
		}
		
		if(!found)
		{
			if(inDelim)
			{
				count++;
				inDelim = false;
			}
		}
		else
		{
			inDelim = true;
		}
	}

	if(count != tokenNum)
		return -1;

	if(i > 0)
		i--;
	//if we get this far, then we've found the start of the token we want
	//now find the end of the token, looking for either a '\0' , '\n', or one of the delimiters

	token[0] = source[i];

	count = 1;
	found = false;
	for(int x = i+count; !found && source[x] != '\n' && source[x] != '\0'; x++)
	{
		for(j = 0; !found && j < numDelimiters; j++)
		{
			if(source[x] == delimiters[j])
				found = true;
		}

		if(!found)
		{
			token[count] = source[x];
			count++;
		}
	}

	token[count] = 0;

	//strncpy(token,source + i,count);

	return i + count - 1 + pre;
}


LineNode* SosUtil::pushStack(LineNode* start, LineNode* newItem)
{
	//if there is nothing in the new Item, don't change start
	if(newItem == 0)
	{
		return start;
	}
		
	newItem->next = start;
	start =  newItem;
	
	return start;
}

void SosUtil::deleteList_Iterative(LineNode* node)
{
    LineNode* temp = 0;
	
    while(node != 0)
    {
		temp = node->next;
		delete node;
		node = temp;
    }
}

PointXY PointXY::operator=(const PointXYLong num)
{
	x = (float)num.x;
	y = (float)num.y;
	return *this;
}

LineNode* SosUtil::deleteNode(LineNode* start,LineNode* nodeToDelete)
{    
    if(nodeToDelete != 0)
    {	
		if(nodeToDelete->prev != 0 && nodeToDelete->next != 0)
		{
			nodeToDelete->prev->next = nodeToDelete->next;//make node before this one point to the node after this one
			nodeToDelete->next->prev = nodeToDelete->prev;//make the node after this one point tho the node before this one
			
			delete nodeToDelete;
			return start;
		}
		if(nodeToDelete->prev == 0 && nodeToDelete->next == 0)
		{
			delete nodeToDelete;
			return 0;	    
		}
		if(nodeToDelete->next ==0)
		{
			nodeToDelete->prev->next = 0;
			delete nodeToDelete;
			return start;
		}
		if(nodeToDelete->prev == 0)
		{
			start = nodeToDelete->next;
			nodeToDelete->next->prev = 0;
			delete nodeToDelete;
			return start;
		}	
    }
    return start;
}

bool SosUtil::between(double num, double lowerVal, double upperVal)
{
	if((num >= lowerVal && num <= upperVal)||(num <= lowerVal && num >= upperVal))
	{
		return true;
	}

	return false;
}

bool SosUtil::between(long num, long lowerVal, long upperVal)
{
	if((num >= lowerVal && num <= upperVal)||(num <= lowerVal && num >= upperVal))
	{
		return true;
	}

	return false;
}

float SosUtil::midWay(float num1, float num2)
{
	ensureSmaller(num1,num2);

	return num1 + (num2 - num1)/2;
}


double LineXY::getPerpDistTo(float x, float y)
{
    //the formula of a line, Ax + By +C = 0, can be derived from a line using two known points, (x1,y1) and (x2, y2)
    //such that A = y2 - y1, B = x1 - x2, and C = ((x1 * y1)-(x1 * y2)+(x2 * y1)-(x1 * y1))
    //Then to get the perpendicular distance from a point to the line, we just stick X and Y into that formula
    double A = 0, B = 0, C = 0;

	float x1 = pt1.x, x2 = pt2.x, y1 = pt1.y, y2 = pt2.y;

	SosUtil::ensureSmaller(x1,x2);
	SosUtil::ensureSmaller(y1,y2);

	double perpDist = 0;
	double xDbl = x, yDbl = y;
		
    //A = y2 - y1;
   // B = x1 - x2;
    //C = (x2 * y1) - (x1 * y2);

//	perpDist = ((A * xDbl) + (B * yDbl) + C)/sqrt(A * A + B * B);

//	LOG<<"getPerpDistTo(): PerpDist = "<<perpDist;

	perpDist = SosUtil::getDist(x,y,this->pt1.x,pt1.y);
	LineXY perpLine;
	perpLine.setLineByAngle(x,y,this->getAngle() + 90,SosUtil::maxVal(SosUtil::fabs(perpDist),1)*2.5);

	double tempX=0,tempY = 0;
	bool retval = getIntersection(perpLine,tempX,tempY,false);

	if(retval)//if we should return the distance from the line or from the endpoints
	{
		perpDist = SosUtil::getDist(tempX,tempY,(double)x,(double)y);
		LOG<<"getPerpDistTo(): Using perp dist from line";
		return perpDist;
	}

	LOG<<"getPerpDistTo(): Using distance from end point";
	return SosUtil::minVal( SosUtil::getDist(x,y,pt1.x,pt1.y),SosUtil::getDist(x,y,pt2.x,pt2.y));
}

double LineXYLong::getPerpDistTo(long x, long y)
{
    //the formula of a line, Ax + By +C = 0, can be derived from a line using two known points, (x1,y1) and (x2, y2)
    //such that A = y2 - y1, B = x1 - x2, and C = ((x1 * y1)-(x1 * y2)+(x2 * y1)-(x1 * y1))
    //Then to get the perpendicular distance from a point to the line, we just stick X and Y into that formula
    double A = 0, B = 0, C = 0;

	long x1 = pt1.x, x2 = pt2.x, y1 = pt1.y, y2 = pt2.y;

	SosUtil::ensureSmaller(x1,x2);
	SosUtil::ensureSmaller(y1,y2);

	double perpDist = 0;
	double xDbl = x, yDbl = y;
		
    //A = y2 - y1;
   // B = x1 - x2;
    //C = (x2 * y1) - (x1 * y2);

//	perpDist = ((A * xDbl) + (B * yDbl) + C)/sqrt(A * A + B * B);

//	LOG<<"getPerpDistTo(): PerpDist = "<<perpDist;
	perpDist = SosUtil::getDist(x,y,this->pt1.x,pt1.y);
	LineXYLong perpLine;
	perpLine.setLineByAngle(x,y,this->getAngle() + 90,SosUtil::maxVal(SosUtil::fabs(perpDist),1)*2.5);

	double tempX=0,tempY = 0;
	bool retval = getIntersection(perpLine,tempX,tempY,false);

	if(retval)//if we should return the distance from the line or from the endpoints
	{
		perpDist = SosUtil::getDist(tempX,tempY,(double)x,(double)y);
		LOG<<"getPerpDistTo(): Using perp dist from line";
		return perpDist;
	}

	LOG<<"getPerpDistTo(): Using distance from end point";
	return SosUtil::minVal( SosUtil::getDist(x,y,pt1.x,pt1.y),SosUtil::getDist(x,y,pt2.x,pt2.y));
	
}

double LineXY::getAngle()
{
	double angle = 0;
	long angleTranslate = 0;
	float x1 = pt1.x, y1 = pt1.y, x2 = pt2.x, y2 = pt2.y;
	if(y1 == y2 && x1 != x2)
	{
		angle = 0;
	}
	else if(x1 == x2 && y1 != y2)
	{
		angle = 90;
	}
	else
	{
		//    if(x1 != x2 && y1 != y2)
		angle = SosUtil::atan2(y2 - y1, x2 - x1);
		
		//only have an accuracy of 3 decimal places
		angleTranslate = long(angle * 1000);
		angle = angleTranslate;
		angle /= 1000;
	}
	while(angle < 0)
	{
		angle += 180;
	}

	while(angle >= 180)
	{
		angle -= 180;
	}
	return angle;
}

double LineXYLong::getAngle()
{
	double angle = 0;
	long angleTranslate = 0;
	long x1 = pt1.x, y1 = pt1.y, x2 = pt2.x, y2 = pt2.y;
	if(y1 == y2 && x1 != x2)
	{
		angle = 0;
	}
	else if(x1 == x2 && y1 != y2)
	{
		angle = 90;
	}
	else
	{
		//    if(x1 != x2 && y1 != y2)
		angle = SosUtil::atan2(y2 - y1, x2 - x1);
		
		//only have an accuracy of 3 decimal places
		angleTranslate = long(angle * 1000);
		angle = angleTranslate;
		angle /= 1000;
	}
	while(angle < 0)
	{
		angle += 180;
	}

	while(angle >= 180)
	{
		angle -= 180;
	}
	return angle;
}

void LineXY::setLineByAngle(double midPointX, double midPointY, double LineAngle, double length )
{
    double x = 0, y = 0;
	
//	LOreport<<"\nIn setLineByAngle("<<midPointX<<','<<midPointY<<','<<length<<','<<LineAngle<<endl;

	while(LineAngle >= 180)
	{
		LineAngle -= 180;
	}
	while(LineAngle <0)
	{
		LineAngle += 180;
	}
	
    x = length/2;
    y = 0;    
    SosUtil::pointRotate(&x,&y,-LineAngle);    
	
	//translate the point by the distance and direction of the first point on the line
	//from the position (0,0)
	pt1.x = float(x + midPointX);
	pt1.y = float(y + midPointY);
	
	x = length/2;
	y = 0;	
	SosUtil::pointRotate(&x,&y,-(LineAngle - 180));
	
	//translate the point by the distance and direction of the first point on the line
	//from the position (0,0)
	pt2.x = float(x + midPointX);
	pt2.y = float(y + midPointY);	
}

void LineXYLong::setLineByAngle(double midPointX, double midPointY, double LineAngle, double length )
{
    double x = 0, y = 0;
	
//	LOreport<<"\nIn setLineByAngle("<<midPointX<<','<<midPointY<<','<<length<<','<<LineAngle<<endl;

	while(LineAngle >= 180)
	{
		LineAngle -= 180;
	}
	while(LineAngle <0)
	{
		LineAngle += 180;
	}
	
    x = length/2;
    y = 0;    
    SosUtil::pointRotate(&x,&y,-LineAngle);    
	
	//translate the point by the distance and direction of the first point on the line
	//from the position (0,0)
	pt1.x = long(x + midPointX);
	pt1.y = long(y + midPointY);
	
	x = length/2;
	y = 0;	
	SosUtil::pointRotate(&x,&y,-(LineAngle - 180));
	
	//translate the point by the distance and direction of the first point on the line
	//from the position (0,0)
	pt2.x = long(x + midPointX);
	pt2.y = long(y + midPointY);	
}

bool LineXY::getIntersection(LineXY line2, double &x, double &y, bool linesAreInfinite, double threshold )
{
//	LOG<<"getIntersection("<<line2.pt1.x<<","<<line2.pt1.y<<") -> ("<<line2.pt2.x<<","<<line2.pt2.y<<")";
	double newX = 0, newY = 0;
	float x1 = pt1.x, y1 = pt1.y, x2 = pt2.x, y2 = pt2.y;
	//    double A = 0, B = 0, C = 0;
    double A1 = 0, B1 = 0, C1 = 0;
    double A2 = 0, B2 = 0, C2 = 0;
    double pointDist1 = 0, pointDist2 = 0;
	
    if(getAngle() == line2.getAngle())
    {
		return false;
    }
	
	//newX and newY are the points of intersection of the two lines

    A1 = y2 - y1;
    B1 = x1 - x2;
    C1 = ((x1 * y1) - (x1 * y2) + (x2 * y1) - (x1 * y1));
	
    A2 = line2.pt2.y - line2.pt1.y;
    B2 = line2.pt1.x - line2.pt2.x;
    C2 = ((line2.pt1.x * line2.pt1.y) - (line2.pt1.x * line2.pt2.y) + 
		(line2.pt2.x * line2.pt1.y) - (line2.pt1.x * line2.pt1.y));
	
    if(((A1 * B2) - (A2 * B1)) != 0)
    {
		newX = ((-C1 * B2) + (C2 * B1))/((A1 * B2) - (A2 * B1));
    }
	
    if(((A1 * B2) - (A2 * B1)) != 0)
    {
		newY = ((-A1 * C2) + (A2 * C1))/((A1 * B2) - (A2 * B1));
    }
	
		
    //now check if the point (newX, newY) is between the points delimiting the
    //ends of both Lines, only if the user has not set the linesAreInfinite = true
	
    LineXY tempLine;
	
	if(!linesAreInfinite )
	{
		pointDist1 = SosUtil::getDist((float)newX,(float)newY,x1,y1);
		
		pointDist2 = SosUtil::getDist((float)newX, (float)newY, x2, y2);
				
		//if the sum of the distance from the point of intersection to either of the two endpoints
		//is longer than the length of the line, then they do not intersect
		if(fabs(pointDist1 + pointDist2 -  SosUtil::getDist(pt1.x,pt1.y,pt2.x,pt2.y)) >threshold )
		{	
			LOG<<"getIntersection returning false 1";
			return false;
		}		


		//tempLine.setPoints((float)newX, (float)newY, line2.pt1.x, line2.pt1.y);
		pointDist1 = SosUtil::getDist((float)newX, (float)newY, line2.pt1.x, line2.pt1.y);
		
		//tempLine.setPoints((float)newX, (float)newY, line2.pt2.x, line2.pt2.y);
		pointDist2 = SosUtil::getDist((float)newX, (float)newY, line2.pt2.x, line2.pt2.y);
		
		if(fabs(pointDist1 + pointDist2 -  SosUtil::getDist(line2.pt1.x,line2.pt1.y,line2.pt2.x,line2.pt2.y)) >threshold )
		{
			/*
			LOG<<"getIntersection returning false for intersection with line "<<line2;
			
			LOG<<"\tpointDist1 = "<<pointDist1<<", pointDist2 = "<<pointDist2;
			LOG<<"\tLength of the other line is = "<<SosUtil::getDist(line2.pt1.x,line2.pt1.y,line2.pt2.x,line2.pt2.y);
			LOG<<"\tDist from line = "<<fabs(pointDist1 + pointDist2 -  SosUtil::getDist(line2.pt1.x,line2.pt1.y,line2.pt2.x,line2.pt2.y));
			LOG<<"\tThreshold = "<<threshold;
			*/
			return false;
		}
	}
	
    x = newX;
    y = newY;
	LOG<<"getIntersection returning true";
    return true;
	
}

bool LineXYLong::getIntersection(LineXYLong line2, double &x, double &y, bool linesAreInfinite )
{
	//    double Line2Slope = 0;
	//    double thisSlope = 0;
    double newX = 0, newY = 0;
	long x1 = pt1.x, y1 = pt1.y, x2 = pt2.x, y2 = pt2.y;
	//    double A = 0, B = 0, C = 0;
    double A1 = 0, B1 = 0, C1 = 0;
    double A2 = 0, B2 = 0, C2 = 0;
    double pointDist1 = 0, pointDist2 = 0;
	//    double rightAngle = 0;
	
    if(getAngle() == line2.getAngle())
    {
		return false;
    }
	
    A1 = y2 - y1;
    B1 = x1 - x2;
    C1 = ((x1 * y1) - (x1 * y2) + (x2 * y1) - (x1 * y1));
	
    A2 = line2.pt2.y - line2.pt1.y;
    B2 = line2.pt1.x - line2.pt2.x;
    C2 = ((line2.pt1.x * line2.pt1.y) - (line2.pt1.x * line2.pt2.y) + 
		(line2.pt2.x * line2.pt1.y) - (line2.pt1.x * line2.pt1.y));
	
    if(((A1 * B2) - (A2 * B1)) != 0)
    {
		newX = ((-C1 * B2) + (C2 * B1))/((A1 * B2) - (A2 * B1));
    }
	
    if(((A1 * B2) - (A2 * B1)) != 0)
    {
		newY = ((-A1 * C2) + (A2 * C1))/((A1 * B2) - (A2 * B1));
    }
	
		
    //now check if the point (newX, newY) is between the points delimiting the
    //ends of both Lines, only if the user has not set the linesAreInfinite = true
	
    LineXYLong tempLine;
	
	if(!linesAreInfinite )
	{
		tempLine.setPoints((long)newX,(long) newY, x1, y1);
		pointDist1 = SosUtil::getDist(tempLine.pt1.x,tempLine.pt1.y,tempLine.pt2.x,tempLine.pt2.y);
		
		tempLine.setPoints((long)newX, (long)newY, x2, y2);
		pointDist2 = SosUtil::getDist(tempLine.pt1.x,tempLine.pt1.y,tempLine.pt2.x,tempLine.pt2.y);
		
		
		if(fabs(pointDist1 + pointDist2 -  SosUtil::getDist(pt1.x,pt1.y,pt2.x,pt2.y)) >2)
		{	
			return false;
		}
		
		tempLine.setPoints((long)newX, (long)newY, line2.pt1.x, line2.pt1.y);
		pointDist1 = SosUtil::getDist(tempLine.pt1.x,tempLine.pt1.y,tempLine.pt2.x,tempLine.pt2.y);
		
		tempLine.setPoints((long)newX, (long)newY, line2.pt2.x, line2.pt2.y);
		pointDist2 = SosUtil::getDist(tempLine.pt1.x,tempLine.pt1.y,tempLine.pt2.x,tempLine.pt2.y);
		
		if(fabs(pointDist1 + pointDist2 -  SosUtil::getDist(line2.pt1.x,line2.pt1.y,line2.pt2.x,line2.pt2.y)) >2)
		{
			return false;
		}
	}
	
    x = newX;
    y = newY;
    return true;
	
}


//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------

//Definition of the LineObj Class

LineObj::LineObj()
{
    reset();    
}

void LineObj::reset()
{
    x1 =  y1 = x2 = y2 = 0;
    angle = 0;
	
}

bool LineObj::getSlope(double &retVal)
{
    double slope = 0;
	
    if(x1 == x2)
    {
		return false;
    }
    else
    {
		slope = (y2 - y1)/(x2 - x1);
		if(slope > 100)
			return false;
		else
			retVal = slope;
		
		return true;
    }
}



void LineObj::setLine(double X1, double Y1, double X2, double Y2)
{
	long angleTranslate;

    x1 = X1;
    y1 = Y1;
    x2 = X2;
    y2 = Y2;	
	
	if(y1 == y2 && x1 != x2)
	{
		angle = 0;
	}
	else if(x1 == x2 && y1 != y2)
	{
		angle = 90;
	}
	else
	{
		//    if(x1 != x2 && y1 != y2)
		angle = SosUtil::atan2(y2 - y1, x2 - x1);
		
		//only have an accuracy of 3 decimal places
		angleTranslate = long(angle * 1000);
		angle = angleTranslate;
		angle /= 1000;
	}
	while(angle < 0)
	{
		angle += 180;
	}

	while(angle >= 180)
	{
		angle -= 180;
	}
	

}

void LineObj::setLineByAngle(double midPointX, double midPointY, double LineAngle, double length )
{
    double x = 0, y = 0;
	
//	LOreport<<"\nIn setLineByAngle("<<midPointX<<','<<midPointY<<','<<length<<','<<LineAngle<<endl;

	while(LineAngle >= 180)
	{
		LineAngle -= 180;
	}
	while(LineAngle <0)
	{
		LineAngle += 180;
	}
	
    x = length/2;
    y = 0;
    angle = LineAngle;
    SosUtil::pointRotate(&x,&y,-LineAngle);
    
	
	//translate the point by the distance and direction of the first point on the line
	//from the position (0,0)
	x1 = x + midPointX;
	y1 = y + midPointY;
	
	x = length/2;
	y = 0;
	angle = LineAngle - 180;
	SosUtil::pointRotate(&x,&y,-(LineAngle - 180));
	
	//translate the point by the distance and direction of the first point on the line
	//from the position (0,0)
	x2 = x + midPointX;
	y2 = y + midPointY;
	
	angle = LineAngle;    
}


void LineObj::setLineAsBisector(double x1, double y1, double x2,double y2, double length)
{
	LineObj tempLine;
	double midpointX = 0, midpointY = 0;

	tempLine.setLine(x1, y1, x2, y2);

	tempLine.getMidPoint(midpointX, midpointY);

	this->setLineByAngle(midpointX, midpointY, tempLine.getAngle() + 90,length);

	
}

void LineObj::getMidPoint(double &x, double& y)
{
    x = (x1 +x2)/2;
    y = (y1 +y2)/2;
}

double LineObj::getAngleBetween(LineObj Line2)
{
    double bigAngle = 0, smallAngle = 0, tempAngle = 0;
	
	//    cout<<"\nIn getAngleBetween(), and the two Lines are \n"<<*this<<endl<<Line2<<endl;
	
    bigAngle = angle;
    smallAngle = Line2.angle;
	
    while(bigAngle >= 180)
    {
		bigAngle -= 180;
    }
	
    while(smallAngle >= 360)
    {
		smallAngle -= 360;
    }
	
    while(smallAngle < 0)
    {
		smallAngle += 180;
    }
	
    while(bigAngle < 0 )
    {
		bigAngle += 180;
    }
	
    if(smallAngle > bigAngle)
    {
		tempAngle = smallAngle;
		smallAngle = bigAngle;
		bigAngle = tempAngle;
		tempAngle = 0;
    }
	
	//    cout<<"\nBig Angle = "<<bigAngle<<" degrees and smallAngle = "<<smallAngle<<endl;
	
    if(fabs(bigAngle - smallAngle) < fabs(bigAngle - (smallAngle +180)))
    {
		tempAngle = fabs(bigAngle -  smallAngle);
    } 
    else
    {
		tempAngle = fabs(bigAngle -  (smallAngle + 180));
    }
	
	//    cout<<"\ngetAngleBetween() returning "<<tempAngle<<endl;
    
    return tempAngle;
	
}


void LineObj::translate(double x, double y, double rotAngle)
{
    double xMid = 0, yMid = 0;
	
    getMidPoint(xMid, yMid);
	
    xMid += x;
    yMid += y;
	
    angle += rotAngle;
	
    while(angle >= 180)
    {
		angle -= 180;
    }
	
    while(angle < 0)
    {
		angle += 180;
    }
	
    setLineByAngle(xMid, yMid, angle, getLength());
	
	
	
}


bool LineObj::getIntersection(LineObj Line2, double &x, double &y, bool linesAreInfinite )
{
	//    double Line2Slope = 0;
	//    double thisSlope = 0;
    double newX = 0, newY = 0;
	//    double A = 0, B = 0, C = 0;
    double A1 = 0, B1 = 0, C1 = 0;
    double A2 = 0, B2 = 0, C2 = 0;
    double pointDist1 = 0, pointDist2 = 0;
	//    double rightAngle = 0;
	
    if(angle == Line2.getAngle())
    {
		return false;
    }
	
    A1 = y2 - y1;
    B1 = x1 - x2;
    C1 = ((x1 * y1) - (x1 * y2) + (x2 * y1) - (x1 * y1));
	
    A2 = Line2.getY2() - Line2.getY1();
    B2 = Line2.getX1() - Line2.getX2();
    C2 = ((Line2.getX1() * Line2.getY1()) - (Line2.getX1() * Line2.getY2()) + (Line2.getX2() * Line2.getY1()) - (Line2.getX1() * Line2.getY1()));
	
    if(((A1 * B2) - (A2 * B1)) != 0)
    {
		newX = ((-C1 * B2) + (C2 * B1))/((A1 * B2) - (A2 * B1));
    }
	
    if(((A1 * B2) - (A2 * B1)) != 0)
    {
		newY = ((-A1 * C2) + (A2 * C1))/((A1 * B2) - (A2 * B1));
    }
	
		
    //now check if the point (newX, newY) is between the points delimiting the
    //ends of both Lines, only if the user has not set the linesAreInfinite = true
	
    LineObj tempLine;
	
	if(!linesAreInfinite )
	{
		tempLine.setLine(newX, newY, x1, y1);
		pointDist1 = tempLine.getLength();
		
		tempLine.setLine(newX, newY, x2, y2);
		pointDist2 = tempLine.getLength();
		
		
		if(fabs(pointDist1 + pointDist2 -  this->getLength()) >2)
		{	
			return false;
		}
		
		tempLine.setLine(newX, newY, Line2.getX1(), Line2.getY1());
		pointDist1 = tempLine.getLength();
		
		tempLine.setLine(newX, newY, Line2.getX2(), Line2.getY2());
		pointDist2 = tempLine.getLength();	
		
		if(fabs(pointDist1 + pointDist2 -  Line2.getLength()) >2)
		{
			return false;
		}
	}
	
    x = newX;
    y = newY;
    return true;
	
}


double LineObj::getLength()
{
    return sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));
}

double LineObj::getPerpDistTo(double x, double y)
{
    //the formula of a line, Ax + By +C = 0, can be derived from a line using two known points, (x1,y1) and (x2, y2)
    //such that A = y2 - y1, B = x1 - x2, and C = ((x1 * y1)-(x1 * y2)+(x2 * y1)-(x1 * y1))
    //Then to get the perpendicular distance from a point to the line, we just stick X and Y into that formula
    double A = 0, B = 0, C = 0;
	
    A = y2 - y1;
    B = x1 - x2;
    C = (x2 * y1) - (x1 * y2);
	
    return ((A * x) + (B * y) + C)/sqrt(A * A + B * B);
	
}


void LineObj::getPoints(double &x1Point,double &y1Point, double &x2Point, double &y2Point)
{
	x1Point = x1;
	y1Point = y1;
	x2Point = x2;
	y2Point = y2;
}


double LineObj::getX1()
{
	return x1;
}
double LineObj::getY1()
{
	return y1;
}

double LineObj::getX2()
{
	return x2;
}

double LineObj::getY2()
{
	return y2;
}

double LineObj::getAngle()
{
	return angle;
}

bool LineObj::getYIntercept(double & yIntercept)
{
	double slope;
	if(x1 ==  x2)
		return false;

	getSlope(slope);
	//y = mx +c, so c = y - mx
	yIntercept = y1 - (slope * x1);
	return true;
}

bool LineObj::getYfromX(double x, double &y)
{
	double slope;
	double yIntercept = 0;
	if(x1 == x2)
		return false;

	getSlope(slope);
	getYIntercept(yIntercept);

	y = slope * x + yIntercept;
	return true;


}

bool LineObj::getXfromY(double y, double &x)
{
	double slope;
	double yIntercept = 0;

	if(y1 == y2)
		return false;

	getSlope(slope);
	getYIntercept(yIntercept);
//y = mx +c , so x = (y-c)/m
	x = (y - yIntercept)/slope;
	return true;
}

ostream &operator<<(ostream &out, const LineObj& outLine)
{
    LineObj myLine = outLine;
	
    out<<'('<<myLine.getX1()<<','<<myLine.getY1()<<')'<<'('<<myLine.getX2()<<','<<myLine.getY2()
		<<')'<<'['<<myLine.getAngle()<<']'<<" length = "<<myLine.getLength();
	
    return out;
	
}

PoseRec::PoseRec()
{
	for(int i=0; i<BUCKET_SIZE;i++)
    {
		wasSet[i] = 0;
    }
}

PoseRec::PoseRec(const PoseRec& otherPoseRec)
{
	for(int i=0; i<BUCKET_SIZE;i++)
    {
		wasSet[i] = otherPoseRec.wasSet[i];
    }
}


PoseRec::PoseRec(int x)
{
	for(int i=0; i<BUCKET_SIZE;i++)
    {
		wasSet[i] = 0;
    }
}


const PoseRec& PoseRec::operator=(const PoseRec& otherVar)
{    
	for(int i = 0; i< BUCKET_SIZE; i++)
    {
		this->wasSet[i] = otherVar.wasSet[i];
    }
	
	return *this;
}

bool PoseRec::operator==(const PoseRec& otherVar) const
{
	for(int i = 0; i< BUCKET_SIZE; i++)
    {
		if(this->wasSet[i] != otherVar.wasSet[i])
		{
			return false;
		}
    }
	return true;
	
}

bool PoseRec::operator!=(const PoseRec& otherVar) const
{
	return !(*this == otherVar);
}


ostream &operator<<(ostream& out, PoseRec p)
{   
	for(int i=0; i<BUCKET_SIZE; i++)
    {
		out<<p.wasSet[i]<<' ';
    }
	return out;
}



istream &operator>>(istream& in, PoseRec p)
{
	return in;
}


ostream &operator<<(ostream &out, const PointXY& outPoint)
{
	char buffer[20]={0};
	
	out<<"(";
	
	SosUtil::floatToString(outPoint.x,2,buffer);
	out<<buffer<<",";
	SosUtil::floatToString(outPoint.y,2,buffer);
	out<<buffer<<")";
	return out;
}

ostream &operator<<(ostream &out, const LineXY& outLine)
{
	out<<outLine.pt1<<"->"<<outLine.pt2;
	return out;
}

ostream &operator<<(ostream &out, const PointXYLong& outPoint)
{
	char buffer[20]={0};
	
	out<<"(";
	
	ltoa(outPoint.x,buffer,10);
	out<<buffer<<",";
	ltoa(outPoint.y,buffer,10);
	out<<buffer<<")";
	return out;
}

ostream &operator<<(ostream &out, const LineXYLong& outLine)
{
	out<<outLine.pt1<<"->"<<outLine.pt2;
	return out;
}

ostream &operator<<(ostream &out, const LineXYLayer& outLine)
{
	out<<(LineXY)(outLine)<<" Layer = "<<outLine.layer<<" Value = "<<outLine.value;
	return out;
}