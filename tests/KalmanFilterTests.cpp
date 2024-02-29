//
// Created by Dhairya Gupta on 10/15/23.
//

#include <gtest/gtest.h>
#include "util/filters/KalmanFilter.h"
#include <iostream>

TEST(Kalman, KalmanFilter) {
    int variables = 3;
    Matrix x0(variables * 2, 1);
    Matrix covariance =  Matrix::getIdentityMatrix(variables*2);

    for(int row = 0; row < covariance.rows(); row++) {
        covariance.set(row, row, 2);
    }
//    covariance.set(2,2, 1313.331f);


    std::cout << "COVARIANCE\n" + covariance.toString();

    Matrix control(variables, 1);
    control.set(0, 0, 17);
    control.set(1, 0, 1.4);
    control.set(2, 0, 0.3);


    KalmanFilter testFilter(variables, 0, x0, control, covariance);

    Matrix y(variables*2, 1);
//    y.set(0, 0, 12);
//    y.set(1, 0, 0.9);
std::cout << "Y AS STRING \n";
    std::cout << y.toString();
    std::cout << "\n";


    testFilter.update(y, control, 0.003);
    for(int i = 0; i < 1500; i++) {
        testFilter.update(testFilter.state(), control, 0.003);

//        if(i % 6 == 0) {
//            for(int x = 0; i < y.rows(); x++) {
//                y.set(x, 0, testFilter.state().get(x, 0));
//            }
//        }
    }
//    testFilter.update(y, control, 0.744);
//    testFilter.update(y, control, 2);
//    testFilter.update(y, control, 2);
//
//    testFilter.update(y, control, 100);
    std::cout << "\nFINAL STATE\n";
    std::cout << testFilter.state().toString();
}
