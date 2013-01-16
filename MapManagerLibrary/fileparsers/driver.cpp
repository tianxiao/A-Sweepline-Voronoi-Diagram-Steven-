#include <iostream.h>
#include "../logger/logger.h"
#include "../sosutil/SosUtil.h"
#include "RobotRunFileHelperStage.h"

int main()
{
	GET_FILE_LOG_GLOBAL

	RobotRunFileHelperStage parser;

	parser.setParseLaser(false);
	parser.setParseSonar(true);
	parser.setParsePosition(true);

	bool ret = parser.parseFile("C:\\Workarea\\Robot Runs\\stanford-gates1\\stanford_small.log");

	cout<<"\nParser returned "<<ret<<endl;

	if(!ret)
		cout<<"The error code is :\n"<<parser.getErrorToken();

	parser.resetSonarReadingIterator();

	int numSonars = parser.numSonars();
	SensorReading reading(numSonars);

	SosPose pose;

	LOG<<"There are "<<parser.numSonarReadings()<<" sonar readings";

	int count = 0;

	while(parser.popSonarReading(reading))
	{
		pose = reading.getPose();
		LOG<<count<<":("<<pose.getX()<<","<<pose.getY()<<") ["<<pose.getTh()<<"] "<<reading.getRange(0)<<" "<<reading.getRange(1)<<" "<<reading.getRange(2)<<" "<<reading.getRange(3)<<" "<<reading.getRange(4)<<" "<<reading.getRange(5)<<" "<<reading.getRange(6)<<" "<<reading.getRange(7)<<" "<<reading.getRange(8)<<" "<<reading.getRange(9)<<" "<<reading.getRange(10)<<" "<<reading.getRange(11)<<" "<<reading.getRange(12)<<" "<<reading.getRange(13)<<" "<<reading.getRange(14)<<" "<<reading.getRange(15);
		count++;
	}

	LOG<<endl;

/*
	char str1[] = "1106955333.287 rossum 6665 sonar 00 1106955333.276 16 +00.115 +00.130 +1.5708 +00.155 +00.115 +0.8727 +00.190 +00.080 +0.5236 +00.210 +00.025 +0.1745 +00.210 -00.025 -0.1745 +00.190 -00.080 -0.5236 +00.155 -00.115 -0.8727 +00.115 -00.130 -1.5708 -00.115 -00.130 -1.5708 -00.155 -00.115 -2.2689 -00.190 -00.080 -2.6180 -00.210 -00.025 -2.9671 -00.210 +00.025 +2.9671 -00.190 +00.080 +2.6180 -00.155 +00.115 +2.2689 -00.115 +00.130 +1.5708 16 0.326 0.399 2.299 2.356 1.565 0.830 2.078 0.281 0.223 0.258 0.345 2.748 7.149 7.082 2.225 7.365\0";
	char **str2;//[5][20] = {{0},{0},{0},{0},{0}};
	char delim[] = " \t";

	str2 = new char*[40];

	int x = 0;
	for(x = 0; x< 40; x++)
	{
		str2[x] = new char[20];
	}

	//int ret = SosUtil::getToken(str1,str2,delim,2,2);
	int ret = SosUtil::tokenise(str1,str2,delim,1,1,40);

	cout<<"return value = "<<ret<<" and the token="<<str2<<"::"<<endl;

	cout<<"The tokens are:"<<endl;
	for(x = 0; x< 40; x++)
	{
		cout<<"["<<x<<"]"<<str2[x]<<endl;
	}

	return 0;
*/


/*
	ifstream in;
	in.open("C:\\Workarea\\Robot Runs\\stanford-gates1\\stanford-gates1.log");
	
	if(!in.is_open())
	{
		cout<<"Couldn't open file"<<endl;
		return 0;
	}

	char buffer[2048] = {0};
	char buffer2[2048] = {0};

	ofstream out;
	out.open("C:\\Workarea\\Robot Runs\\stanford-gates1\\stanford_small.log");

	if(!out.is_open())
	{
		cout<<"Could not open output file"<<endl;
		in.close();
		return 0;
	}

	in.getline(buffer,2048);
	while(!in.eof())
	{
	//	SosUtil::trim(buffer,buffer2);
		if(SosUtil::firstInstance(buffer,"#") < 0 && 
			SosUtil::firstInstance(buffer,"sonar",true) > 0)
		{
			out<<buffer<<endl;
		}		

		in.getline(buffer,2048);
	}

	out.close();
	in.close();
*/

	return 0;
}