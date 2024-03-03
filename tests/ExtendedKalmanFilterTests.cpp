//
// Created by Dhairya Gupta on 10/15/23.
//

#include <gtest/gtest.h>

#include <iostream>

#include "util/filters/ExtendedKalmanFilter.h"

TEST(Kalman, ExtendedKalmanFilter) {
    ExtendedKalmanFilter testFilter;

    ControlState control = {50, 1, 0};
    VehicleState gpsEstimate = {0, 0, 0, 0, 0};
    float delta_t = 0.003;

//    testFilter.update(control, gpsEstimate, delta_t);

    for (int i = 0; i < 1500; i++) {
        testFilter.update(control, gpsEstimate, delta_t);
    }

    std::cout << "\nFINAL STATE\n";
    std::cout << "X: " << testFilter.getState().x << "\n";
    std::cout << "Y: " << testFilter.getState().y << "\n";
    std::cout << "V_X: " << testFilter.getState().v_x << "\n";
    std::cout << "V_Y: " << testFilter.getState().v_y << "\n";
    std::cout << "THETA: " << testFilter.getState().theta << "\n";
}