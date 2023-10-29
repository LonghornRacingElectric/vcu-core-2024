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

    covariance.set(0,0, 12.0f);
    covariance.set(1,1, 113.0f);
//    covariance.set(2,2, 1313.331f);


    std::cout << "COVARIANCE\n" + covariance.toString();

    Matrix control(variables, 1);
    control.set(0, 0, 17);
    control.set(1, 0, 12.4);
    control.set(2, 0, 9.3);


    KalmanFilter testFilter(variables, 0, x0, control, covariance);

    Matrix y(variables*2, 1);
    y.set(0, 0, 12);
    y.set(1, 0, 0.9);

    std::cout << y.toString();
    std::cout << "\n";

    testFilter.update(y, control, 3);

    testFilter.update(y, control, 0.744);
//    testFilter.update(y, control, 2);
//    testFilter.update(y, control, 2);
//
//    testFilter.update(y, control, 100);
    std::cout << "\nFINAL STATE\n";
    std::cout << testFilter.state().toString();
}
