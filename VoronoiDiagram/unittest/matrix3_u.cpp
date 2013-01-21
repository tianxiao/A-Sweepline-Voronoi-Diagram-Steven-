#include "stdafx.h"
#include <math.h>
#include "../thirdparty/gtest-1.6.0/include/gtest.h"
//#include "../RealisticRayTracingDLL/Matrix3.h"
//
//TEST(Matrix3, SolveLinear3){
//	//EXPECT_EQ(1,1);
//	txMatrix3 identity;
//	identity.Identity();
//
//	txVec3d result = identity.SolveLinear3(txVec3d(1.0,1.0,1.0));
//	EXPECT_TRUE(result.GetX()-1.0<MATRIX_EPSILON);
//	EXPECT_TRUE(result.GetY()-1.0<MATRIX_EPSILON);
//	EXPECT_TRUE(result.GetZ()-1.0<MATRIX_EPSILON);
//
//	txMatrix3 m(1.0,2.0,3.0,  0.0,1.0,4.0,  5.0,3.0,7.0);
//	txVec3d result1 = m.SolveLinear3(txVec3d(14.0,14.0,32.0));
//	EXPECT_TRUE(abs(result1.GetX()-1.0)<MATRIX_EPSILON);
//	EXPECT_TRUE(abs(result1.GetY()-2.0)<MATRIX_EPSILON);
//	EXPECT_TRUE(abs(result1.GetZ()-3.0)<MATRIX_EPSILON);
//}
//
//TEST(Matrix3, Determin){
//	txMatrix3 identity;
//	identity.Identity();
//	EXPECT_TRUE((identity.Determinant()-1.0)<MATRIX_EPSILON);
//
//	txMatrix3 m(1.0,2.0,3.0,  0.0,1.0,4.0,  5.0,3.0,7.0);
//	EXPECT_TRUE(abs(m.Determinant()-20.0)<MATRIX_EPSILON);
//}
