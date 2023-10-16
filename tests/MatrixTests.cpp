//
// Created by Dhairya Gupta on 10/8/23.
//

#include <gtest/gtest.h>
#include "util/math/Matrix.h"

TEST(Math, MatrixTests) {
Matrix testMatrix1(2, 2, 3);
EXPECT_EQ(3, testMatrix1.get(1, 1));

Matrix testMatrix2 = testMatrix1 * testMatrix1;
EXPECT_EQ(18, testMatrix2.get(1, 1));

auto *testMatrix3 = new Matrix(2,3, 18);
testMatrix3->set(1, 2, 36);
*testMatrix3 = testMatrix3->getTranspose();
EXPECT_EQ(36, testMatrix3->get(2,1));

Matrix testMatrix4(2,2,0);
testMatrix4.set(0,0,12);
testMatrix4.set(1,1,6);

Matrix testMatrix5(2,2,2);

testMatrix5 = testMatrix4 / testMatrix5;

EXPECT_EQ(6, testMatrix5.get(0,0));
}