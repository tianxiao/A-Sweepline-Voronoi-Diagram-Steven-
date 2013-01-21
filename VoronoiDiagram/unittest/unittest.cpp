// unittest.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "../thirdparty/gtest-1.6.0/include/gtest.h"


int _tmain(int argc, _TCHAR* argv[])
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
