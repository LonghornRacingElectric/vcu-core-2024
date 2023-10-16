//
// Created by Dhairya Gupta on 10/15/23.
//

#include <gtest/gtest.h>
#include "util/filters/KalmanFilter.h"
#include <iostream>

TEST(Kalman, KalmanFilter) {
    Matrix x0(6, 1);
    Matrix covariance =  Matrix::getIdentityMatrix(6);

    std::cout << "\nCOVARIANCE OG\n" + covariance.toString();

    covariance.set(0,0, 12.0f);
    covariance.set(1,1, 113.0f);
    covariance.set(2,2, 1313.331f);


    std::cout << "COVARIANCE\n" + covariance.toString();

    Matrix control(3, 1);
    control.set(0, 0, 3);

    KalmanFilter testFilter(3, 0, x0, control, covariance);

    Matrix y(6, 1);
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

    std::cout << testFilter.state().toString();
}
